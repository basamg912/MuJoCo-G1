# MuJoCo-KIST 프로젝트: C++ 기반 로봇 제어 핵심 동작 과정 및 메모리/API 상세 분석

이 문서는 MuJoCo 시뮬레이터를 기반으로 작동하는 로봇(G1 29-DOF) 제어 시스템의 핵심 파이프라인을 분석합니다. 특별히 C++ 메모리 관리 기법(스마트 포인터, 원시 포인터, 동적 할당 등), 주요 라이브러리 API(MuJoCo, Eigen, ONNX Runtime)의 내부 사용 방식, 그리고 메모리 정렬 및 쓰레딩 관점에서의 깊은 추론을 포함하여 매우 상세하게 기술합니다.

(참고: 본 프로젝트 소스 코드 상 `include/rbdl/` 폴더가 존재하나, 실제 동역학/운동학 계산(`CModel::update_dynamics`)에는 RBDL 대신 MuJoCo의 C API 네이티브 함수인 `mj_fullM` 등을 이용해 Eigen 행렬로 변환하여 사용하고 있습니다.)

---

## 1. C++ 메모리 관리 및 시스템 코어 설계 (`src/main.cc`)

제어 루프의 근간은 C++의 메모리 안전성과 C API(MuJoCo) 간의 브릿지 역할을 완벽히 수행하는 데 있습니다.

### 1.1 스마트 포인터와 RAII 패턴 활용
```cpp
auto sim = std::make_unique<mj::Simulate>(
    std::make_unique<mj::GlfwAdapter>(), &cam, &opt, &pert, false
);
```
* **`std::unique_ptr` 및 `std::make_unique`:** `mj::Simulate` 객체의 라이프사이클을 독점적으로 관리합니다. 시뮬레이션 환경 구성은 무거운 리소스(UI 컨텍스트, 렌더링 버퍼 등)를 동반합니다. 스마트 포인터를 사용하면 `main()` 스코프를 벗어나는 즉시 소멸자가 호출되어 자동으로 메모리가 해제되므로 메모리 누수(Memory Leak)를 원천 차단합니다. (RAII: Resource Acquisition Is Initialization)

### 1.2 C 스타일 원시 포인터 (Raw Pointers)와 동적 할당
```cpp
mjModel *m = nullptr;
mjData *d = nullptr;
mjtNum *ctrlnoise = nullptr;
// 할당 및 해제
ctrlnoise = (mjtNum *)malloc(sizeof(mjtNum) * m->nu);
mj_deleteData(d);
mj_deleteModel(m);
free(ctrlnoise);
```
* **원시 포인터의 당위성:** MuJoCo 엔진은 C 언어 구조체 기반의 API입니다. `mj_loadXML`이나 `mj_makeData`는 내부적으로 C 스타일 메모리 블록을 생성하고 그 주소값(`mjModel*`, `mjData*`)을 반환합니다. 이를 C++ 스마트 포인터에 억지로 담기보다는 원시 포인터를 유지하되, `PhysicsLoop` 스레드가 종료되거나 모델 리셋 시 수동으로 `mj_deleteData(d)`와 `free()`를 명시해주는 패턴을 채택했습니다.
* **배열과 `malloc`:** 컨트롤 노이즈를 저장하기 위한 배열(`ctrlnoise`)은 `std::vector` 대신 `malloc`을 사용했습니다. 이는 MuJoCo의 배열 제어 철학(C array 연속 메모리)을 직접적으로 따르며 `m->nu`(액추에이터 갯수)만큼 정확한 메모리 오프셋을 갖도록 하기 위함입니다.

### 1.3 멀티스레딩과 상호 배제 (Locking)
```cpp
std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);
const std::lock_guard<mj::SimulateMutex> lock(sim.mtx);
```
* **스레드 분리 (`std::thread`):** UI 이벤트 및 OpenGL 렌더링(메인 스레드, 블로킹 렌더 루프)과 500Hz/50Hz의 로봇 물리 스텝(`PhysicsLoop`) 연산을 분리합니다. 
* **`std::lock_guard`:** 물리 연산을 수행할 때마다 뮤텍스(`mtx`)를 잠급니다. 렌더링 스레드가 `d`(mjData)의 메모리 블록(로봇 위치, 자코비안 등)을 읽어갈 때 물리 연산 스레드가 `d`를 변경(Race Condition)하는 것을 막아줍니다.

---

## 2. 제어 주기 동기화 및 MuJoCo API 제어 흐름 (`src/main.cc`)

### 2.1 Forward vs Step
```cpp
mj_forward(m, d);
mj_step(m, d);
```
* **`mj_forward(m, d)`:** 시간에 따른 적분(Integration)을 수행하지 **않고**, 현재 상태(`d->qpos`, `d->qvel`)만을 참조하여 운동학적 파생 변수(Body의 좌표, 회전행렬, 자코비안) 및 동역학적 관성 행렬 등을 계산합니다. 로봇이 처음 스폰(Spawn)되었을 때나, 초기 자세 설정 직후 시스템 상태를 갱신하기 위해 사용됩니다.
* **`mj_step(m, d)`:** 실질적으로 시뮬레이션의 시간을 `m->opt.timestep` (보통 0.002초) 만큼 전진시킵니다. 내부적으로 오일러 또는 룽게-쿠타 적분기를 통해 `v_next = v + a*dt`, `q_next = q + v_next*dt` 형태의 계산을 수행합니다. 

### 2.2 Dual-Rate 로직 구현체
```cpp
constexpr int POLICY_DECIMATION = 10;
if (feet_contacted && policy_step++ % POLICY_DECIMATION == 0) {
    Control.control_mujoco(); 
}
Control.step_pd(); 
Control.write(d->ctrl);
```
물리 엔진은 500Hz로 동작하며 매 틱마다 PD 토크(`step_pd()`)를 계산해 `d->ctrl` (MuJoCo 액추에이터 입력 배열)에 값을 씁니다. Policy는 연산 부하가 크고 실세계와의 격차 완화를 위해 `10` 스텝에 한 번씩만(`0.02초, 50Hz`) 추론하여 목표 각도를 갱신합니다.

---

## 3. 동역학 수식 모델링과 Eigen API (`src/robot/robotmodel.cpp`, `controller.cpp`)

로봇 컨트롤러에서는 자체적인 CModel 클래스를 통해 MuJoCo의 C 배열 구조를 C++의 객체 지향 행렬 구조(`Eigen`)로 래핑하여 사용합니다.

### 3.1 `qpos`와 `qvel`의 배열 불일치와 포인터 산술 (Pointer Arithmetic)
```cpp
_k = m->nv - _qvel_offset;
```
* **`qpos` (위치) vs `qvel` (속도):** 플로팅 베이스(자유 관절, `mjJNT_FREE`)를 가진 휴머노이드에서 `qpos`는 위치(x,y,z)와 쿼터니언(qw,qx,qy,qz)으로 7개의 원소를, `qvel`은 선속도(vx,vy,vz)와 각속도(wx,wy,wz)로 6개의 원소를 갖습니다. 
* 코드는 `m->jnt_type[i] == mjJNT_FREE` 조건으로 플로팅 베이스를 건너뛰고 오직 조인트 제어 영역만의 오프셋(`_qvel_offset=6`)을 설정하여 `nv`(총 속도 자유도)에서 이를 뺀 값(`_k=29`)만을 다루도록 철저히 계산합니다.

### 3.2 MuJoCo 관성 행렬 언패킹 (Unpacking)
```cpp
mjtNum M[nv*nv];
mj_fullM(_mj_model, M, _mj_data->qM);
for (int i = 0; i < _k; i++)
    for (int j = 0; j < _k; j++)
        _A(i, j) = M[(i+ _qvel_offset) * nv + (j+ _qvel_offset)];
```
* `d->qM`은 메모리 최적화를 위해 상삼각/하삼각 대칭성을 이용한 1차원 희소(Sparse) 배열로 되어있습니다. 이를 `mj_fullM` API를 이용해 완전한 이차원 배열(C 배열) `M`으로 풀어냅니다. 
* 그 후 앞서 구한 오프셋(`_qvel_offset`)을 더하여 **플로팅 베이스를 제외한 조인트 공간의 관성 행렬(Inertia Matrix)**만 정확하게 추출하여 `Eigen::MatrixXd _A`에 매핑합니다. (이 과정은 통상적인 역동역학 제어에서 아주 중요한 관성 보상을 할 때 쓰입니다.)
* 중력 및 코리올리 보상은 `d->qfrc_bias` 배열 값을 읽어 Eigen 벡터 `_bg`에 저장합니다.

### 3.3 Eigen 최적화 API (PD 제어)
```cpp
_torque = _kp_diag.cwiseProduct(_q_des - _q) + _kd_diag.cwiseProduct(_qdot_des - _qdot);
```
* 배열 반복문을 쓰지 않고, Eigen 라이브러리의 SIMD(Single Instruction Multiple Data) 최적화 연산인 `.cwiseProduct()`(원소별 곱셈)를 사용하여 29개 자유도의 토크를 한 줄의 수식과 병렬 벡터 연산으로 극도로 빠르게 산출합니다.

---

## 4. 인공지능 Policy (ONNX Runtime) C++ 동작 구조 (`src/policy/policy.cpp`)

파이썬 환경에서 학습된 강화학습 모델(`fastsac`)을 C++ 상에서 텐서로 다루는 과정은 메모리 정렬(Memory Alignment) 측면에서 매우 정교하게 구성됩니다.

### 4.1 세션 설정과 메모리 아레나(Arena)
```cpp
Ort::SessionOptions session_options;
session_options.SetIntraOpNumThreads(1);
_session = Ort::Session(_env, model_path.c_str(), session_options);

_memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
```
* **Thread 억제:** `SetIntraOpNumThreads(1)`을 통해 ONNX 추론 스레드를 단일로 제한합니다. 멀티코어 환경에서 짧은 신경망 추론에 여러 스레드가 붙으면 컨텍스트 스위칭 지연이 발생해 Real-time 제어 루틴의 Determinism(결정성)이 무너질 수 있기 때문입니다.
* **Arena Allocator:** `OrtArenaAllocator`는 메모리 풀링(Pooling) 방식입니다. 추론 시 매번 OS 커널에 `malloc` 요청을 보내는 대신, 초기에 한 번 큰 메모리 덩어리(Arena)를 잡아두고 내부적으로 재활용합니다. C++의 실시간 제어 속도를 담보하는 핵심입니다.

### 4.2 C++ 배열의 Tensor Wrapping (포인터 바인딩)
```cpp
std::vector<float> input_data(obs.size());
for (int i=0; i<obs.size(); i++) input_data[i] = static_cast<float>(obs[i]);

Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
    _memory_info, input_data.data(), input_data.size(), input_shape.data(), input_shape.size()
);
```
* **메모리 연속성(Contiguous Memory):** C++의 `std::vector`는 내부 요소가 일렬로 메모리에 배치됨을 보장합니다. `input_data.data()`로 이 연속된 메모리의 첫 시작점(원시 포인터)을 ONNX에 넘깁니다. 
* **Zero-copy Tensor:** `CreateTensor` API는 넘겨받은 메모리 공간을 복사하지 않고 텐서 인터페이스만 씌워서(Wrap) 사용합니다. 단, C++ Eigen의 `double` 타입을 딥러닝 모델(`float32`)로 형변환(`static_cast`)하기 위한 버퍼 역할로 `input_data`가 선행된 것입니다.

### 4.3 추론 실행 및 메모리 언패킹
```cpp
auto output_tensors = _session.Run(Ort::RunOptions{nullptr}, input_name, &input_tensor, 1, output_name, 1);
float* output_data = output_tensors[0].GetTensorMutableData<float>();
```
* 추론이 끝나면 `GetTensorMutableData<float>()`를 통해 ONNX가 결과값을 뱉은 메모리 주소(포인터)를 받아옵니다. C++ 관점에서는 이 주소로부터 시작하는 배열에 인덱싱(`output_data[i]`)하여 `Eigen::VectorXd action` 으로 옮겨 닮습니다.

---

## 5. 요약 

이 로봇 제어 코드는 C++의 두 가지 강력한 특성, 즉 **저수준 하드웨어/메모리 제어 능력**과 **고수준의 추상화(객체/행렬 연산)**를 완벽하게 조화시킨 사례입니다.
1. `malloc`/`free` 및 C API (`mujoco.h`)를 통해 메모리 레이아웃(희소행렬 언패킹, 자유 관절 오프셋 포인터 연산)을 밑바닥부터 다루며,
2. `std::unique_ptr` 및 `std::lock_guard`를 활용해 리소스 누수와 멀티스레드 충돌을 차단하고,
3. `ONNX Runtime`의 포인터 래핑(Tensor Wrapping) 기술과 메모리 아레나(Arena)를 사용해 지연 없는 AI 모델 추론을 수행하여,
4. 최종적으로는 `Eigen` 라이브러리의 수식적 간결함과 SIMD 최적화를 거쳐 관절의 토크를 도출해냅니다.
