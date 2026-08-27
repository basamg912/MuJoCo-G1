# macOS 포팅 가이드 (Linux → macOS Apple Silicon)

이 문서는 원래 Linux x86-64 기반으로 작성된 MuJoCo 시뮬레이션 프로젝트를  
macOS Apple Silicon(arm64)에서 빌드하고 실행할 수 있도록 포팅한 과정을 상세하게 기록한다.

- **포팅 날짜**: 2026-04-16
- **브랜치**: `feature/macos`
- **타겟 아키텍처**: arm64 (Apple M 시리즈)
- **macOS 버전**: macOS 26.4.1

---

## 목차

1. [포팅 전 상황 분석](#1-포팅-전-상황-분석)
2. [Linux vs macOS 핵심 차이점](#2-linux-vs-macos-핵심-차이점)
3. [Step 1 — MuJoCo 라이브러리 교체](#3-step-1--mujoco-라이브러리-교체)
4. [Step 2 — ONNX Runtime 교체](#4-step-2--onnx-runtime-교체)
5. [Step 3 — GLFW 설치](#5-step-3--glfw-설치)
6. [Step 4 — RBDL 소스 빌드](#6-step-4--rbdl-소스-빌드)
7. [Step 5 — CMakeLists.txt 수정](#7-step-5--cmakeliststxt-수정)
8. [Step 6 — 코드 버그 수정](#8-step-6--코드-버그-수정)
9. [Step 7 — dylib install name 수정](#9-step-7--dylib-install-name-수정)
10. [최종 디렉토리 구조](#10-최종-디렉토리-구조)
11. [빌드 방법](#11-빌드-방법)
12. [트러블슈팅](#12-트러블슈팅)

---

## 1. 포팅 전 상황 분석

### 원본 프로젝트의 의존성

```
MuJoCo-kist/
├── mujoco-3.6.0/
│   └── lib/
│       ├── libmujoco.so          ← Linux x86-64 전용
│       └── libmujoco.so.3.6.0   ← Linux x86-64 전용
├── onnxruntime/
│   ├── lib/
│   │   ├── libonnxruntime.so    ← Linux x86-64 전용
│   │   └── ...
│   └── include/
└── CMakeLists.txt               ← Linux 전용 설정
```

```bash
# 확인 방법: file 명령어로 아키텍처 확인
file mujoco-3.6.0/lib/libmujoco.so
# → ELF 64-bit LSB shared object, x86-64  (macOS에서 실행 불가)
```

---

## 2. Linux vs macOS 핵심 차이점

포팅 과정에서 처리해야 할 차이점들이다.

### 2-1. 공유 라이브러리 확장자

| 항목 | Linux | macOS |
|------|-------|-------|
| 공유 라이브러리 확장자 | `.so` | `.dylib` |
| 라이브러리 로딩 방식 | `LD_LIBRARY_PATH` | `DYLD_LIBRARY_PATH` 또는 rpath |
| 라이브러리 경로 확인 | `ldd` | `otool -L` |
| 라이브러리 내부 경로 확인 | `readelf -d` | `otool -D` |
| 경로 수정 | `patchelf` | `install_name_tool` |

### 2-2. 그래픽 서브시스템

| 항목 | Linux | macOS |
|------|-------|-------|
| 디스플레이 서버 | X11 (Xorg) | Quartz/Cocoa |
| OpenGL 진입점 | EGL, GLX | CGL (Core OpenGL) |
| 디스플레이 동기화 | X11 sync | CoreVideo (`CVDisplayLink`) |
| GUI 프레임워크 | GTK/Qt/X11 | Cocoa (AppKit) |

MuJoCo simulate 코드는 이 차이를 `.mm` 파일로 처리한다:
- `glfw_corevideo.mm` — CoreVideo 기반 vsync 구현
- `macos_gui.mm` — macOS Cocoa GUI 처리

`.mm` 확장자는 **Objective-C++** 파일이다. C++ 코드 안에서 Objective-C API(Apple 네이티브 API)를 함께 사용할 수 있다.

### 2-3. 링커 플래그

| 플래그 | Linux | macOS |
|--------|-------|-------|
| `-Wl,-no-as-needed` | 사용 가능 | **에러** (ld64 미지원) |
| `-lGL` | 사용 가능 | 불필요 (framework 사용) |
| `OpenGL::EGL` | 사용 가능 | **없음** |
| `-framework Cocoa` | 없음 | macOS 전용 |
| `-framework CoreVideo` | 없음 | macOS 전용 |
| `-framework IOKit` | 없음 | macOS 전용 |

### 2-4. rpath vs LD_LIBRARY_PATH

Linux에서는 실행 시 `LD_LIBRARY_PATH`로 라이브러리 경로를 지정하거나,  
`/etc/ld.so.conf`에 등록한다.

macOS에서는 바이너리 내부에 **rpath(Run-Path)**를 직접 박아넣는다:
```bash
# 바이너리에 박힌 rpath 확인
otool -l ./run | grep -A3 LC_RPATH
```

CMakeLists.txt에서 rpath를 설정하는 방법:
```cmake
set_target_properties(run PROPERTIES
    BUILD_RPATH   "경로1;경로2"   # 빌드 시 사용
    INSTALL_RPATH "경로1;경로2"   # 설치 후 사용
)
```

### 2-5. install name (macOS 전용 개념)

macOS의 `.dylib`는 내부에 **install name**이라는 자신의 경로를 기록한다.  
다른 바이너리가 이 dylib를 링크하면, 그 install name을 참조 경로로 사용한다.

```bash
# install name 확인
otool -D libmujoco.3.6.0.dylib
# → @rpath/mujoco.framework/Versions/A/libmujoco.3.6.0.dylib  (원본)
# → @rpath/libmujoco.3.6.0.dylib  (수정 후)

# install name 변경
install_name_tool -id "@rpath/libmujoco.3.6.0.dylib" libmujoco.3.6.0.dylib

# 다른 dylib가 참조하는 경로 변경
install_name_tool -change \
    "@rpath/mujoco.framework/Versions/A/libmujoco.3.6.0.dylib" \
    "@rpath/libmujoco.3.6.0.dylib" \
    libsdf_plugin.dylib
```

`@rpath`는 실행 파일에 박힌 rpath 목록을 순서대로 탐색하라는 의미다.  
`@executable_path`는 실행 파일이 있는 디렉토리를 의미한다.

---

## 3. Step 1 — MuJoCo 라이브러리 교체

### 문제

기존 `mujoco-3.6.0/lib/` 에는 Linux용 `.so` 파일만 있었다.

### 해결 과정

MuJoCo 공식 GitHub에서 macOS 전용 DMG를 다운로드한다.

```bash
# 다운로드
curl -L "https://github.com/google-deepmind/mujoco/releases/download/3.6.0/mujoco-3.6.0-macos-universal2.dmg" \
     -o /tmp/mujoco-3.6.0-macos.dmg

# DMG 마운트
hdiutil attach /tmp/mujoco-3.6.0-macos.dmg -nobrowse
# → /Volumes/MuJoCo 에 마운트됨

# 아키텍처 확인 (universal2 = arm64 + x86_64)
file /Volumes/MuJoCo/mujoco.framework/Versions/A/libmujoco.3.6.0.dylib
# → Mach-O universal binary with 2 architectures: [x86_64] [arm64]
```

**DMG 내부 구조:**
```
/Volumes/MuJoCo/
├── mujoco.framework/
│   └── Versions/A/
│       ├── libmujoco.3.6.0.dylib   ← 메인 라이브러리
│       └── Headers/                ← C 헤더 파일들
├── MuJoCo.app/
│   └── Contents/
│       └── MacOS/
│           └── mujoco_plugin/      ← 플러그인 dylib들
└── simulate/
    ├── glfw_adapter.cc
    ├── glfw_corevideo.mm           ← macOS 전용 (새로 추가)
    ├── macos_gui.mm                ← macOS 전용 (새로 추가)
    └── ...
```

```bash
# 1. 메인 dylib 복사
cp /Volumes/MuJoCo/mujoco.framework/Versions/A/libmujoco.3.6.0.dylib \
   mujoco-3.6.0/lib/

# 2. symlink 생성 (libmujoco.dylib → libmujoco.3.6.0.dylib)
ln -sf libmujoco.3.6.0.dylib mujoco-3.6.0/lib/libmujoco.dylib

# 3. 플러그인 dylib 복사
cp /Volumes/MuJoCo/MuJoCo.app/Contents/MacOS/mujoco_plugin/*.dylib \
   mujoco-3.6.0/bin/mujoco_plugin/

# 4. macOS 전용 simulate 소스 복사
cp /Volumes/MuJoCo/simulate/glfw_corevideo.h   mujoco-3.6.0/simulate/
cp /Volumes/MuJoCo/simulate/glfw_corevideo.mm  mujoco-3.6.0/simulate/
cp /Volumes/MuJoCo/simulate/macos_gui.mm       mujoco-3.6.0/simulate/

# 5. 헤더 업데이트
cp -r /Volumes/MuJoCo/mujoco.framework/Versions/A/Headers/* \
      mujoco-3.6.0/include/mujoco/

# 6. DMG 언마운트
hdiutil detach /dev/disk7
```

---

## 4. Step 2 — ONNX Runtime 교체

### 문제

기존 `onnxruntime/` 디렉토리에는 Linux x86-64용 `.so` 파일이 들어있었다.

```bash
file onnxruntime/lib/libonnxruntime.so
# → ELF 64-bit LSB shared object, x86-64
```

### 해결 과정

```bash
# 기존 Linux 버전 백업
mv onnxruntime onnxruntime-linux-x64

# macOS arm64 버전 다운로드 (버전 통일: 1.24.4)
curl -L "https://github.com/microsoft/onnxruntime/releases/download/v1.24.4/onnxruntime-osx-arm64-1.24.4.tgz" \
     -o /tmp/onnxruntime-osx-arm64-1.24.4.tgz

# 압축 해제 후 프로젝트에 배치
tar xzf /tmp/onnxruntime-osx-arm64-1.24.4.tgz -C /tmp/
cp -r /tmp/onnxruntime-osx-arm64-1.24.4 onnxruntime
```

```bash
# 확인
file onnxruntime/lib/libonnxruntime.dylib
# → Mach-O 64-bit dynamically linked shared library arm64

otool -D onnxruntime/lib/libonnxruntime.dylib
# → @rpath/libonnxruntime.1.24.4.dylib  (이미 @rpath 기반으로 올바르게 설정됨)
```

---

## 5. Step 3 — GLFW 설치

### 문제

기존 CMakeLists.txt에서 `libglfw.so.3`을 직접 경로로 찾고 있었다:

```cmake
# 기존 (Linux 전용)
find_library(GLFW libglfw.so.3 HINTS ${CMAKE_SOURCE_DIR}/mujoco-3.6.0/bin)
```

### 해결 과정

macOS에서는 Homebrew로 GLFW를 설치하고 CMake find_package를 활용한다.

```bash
brew install glfw
```

설치 경로: `/opt/homebrew/Cellar/glfw/3.4/`

```bash
# 확인
file /opt/homebrew/opt/glfw/lib/libglfw.dylib
# → Mach-O 64-bit dynamically linked shared library arm64
```

CMakeLists.txt에서의 변경:
```cmake
# 기존
find_library(GLFW libglfw.so.3 HINTS ...)

# 변경 후
find_package(glfw3 REQUIRED HINTS /opt/homebrew/lib/cmake/glfw3)
# 링크 시: glfw (cmake target 이름)
```

> **주의**: `find_package(GLFW3)` 와 `find_package(glfw3)` 는 다르다.  
> Homebrew가 설치하는 cmake config 파일명이 `glfw3Config.cmake` 이므로  
> `find_package(glfw3)` 를 사용해야 한다.  
> cmake target 이름은 `glfw` (대소문자 구분).

---

## 6. Step 4 — RBDL 소스 빌드

### 문제

RBDL(Rigid Body Dynamics Library)은 Homebrew 패키지가 없어서 직접 빌드해야 한다.  
또한 시스템에 Eigen 5.0.1이 기본으로 잡히는데, Eigen 5.x는 RBDL과 호환되지 않는다.

**Eigen 5.x 호환 불가 에러:**
```
error: no member named '_check_template_params' in 'Eigen::Matrix<double, 6, 6>'
```
이 멤버 함수가 Eigen 최신 버전에서 제거되었기 때문이다.

### 해결 과정

#### Eigen 버전 확인

```bash
# Homebrew에 3.x와 5.x가 동시에 설치되어 있음
brew info eigen
# → /opt/homebrew/Cellar/eigen/3.4.0_1  (3.x)
# → /opt/homebrew/Cellar/eigen/5.0.1    (5.x, 기본)

# RBDL 빌드 시 3.4.0 경로를 명시해야 함
EIGEN3_PATH=/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3
```

#### RBDL 소스 클론 및 빌드

```bash
# 소스 클론 (서브모듈 포함 — urdfreader에 필요)
git clone https://github.com/rbdl/rbdl.git /tmp/rbdl-src --depth=1
cd /tmp/rbdl-src
git submodule update --init --recursive
# → addons/urdfreader/thirdparty/urdfparser 서브모듈 초기화

# 빌드 디렉토리 생성
mkdir /tmp/rbdl-build && cd /tmp/rbdl-build

# CMake 구성
cmake /tmp/rbdl-src \
  -DCMAKE_BUILD_TYPE=Release \
  -DRBDL_BUILD_ADDON_URDFREADER=ON \
  -DRBDL_BUILD_STATIC=ON \
  -DEIGEN3_INCLUDE_DIR=/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3

# 빌드
make -j$(sysctl -n hw.ncpu)
```

**빌드 결과물:**
```
/tmp/rbdl-build/
├── librbdl.a                            ← RBDL 정적 라이브러리
└── addons/urdfreader/
    └── librbdl_urdfreader.a             ← URDF 파서 정적 라이브러리
```

#### 프로젝트에 배치

`sudo` 권한 없이 `/usr/local`에 설치하는 대신, 프로젝트 내부에 직접 복사한다.

```bash
# 정적 라이브러리 복사
mkdir -p lib/
cp /tmp/rbdl-build/librbdl.a                           lib/
cp /tmp/rbdl-build/addons/urdfreader/librbdl_urdfreader.a  lib/

# 헤더 복사
mkdir -p include/rbdl/
cp -r /tmp/rbdl-src/include/rbdl/*           include/rbdl/
cp /tmp/rbdl-src/addons/urdfreader/urdfreader.h  include/rbdl/
```

> **정적 라이브러리(`.a`)를 선택한 이유**:  
> 동적 라이브러리(`.dylib`)는 실행 시 경로를 맞춰줘야 하지만,  
> 정적 라이브러리는 빌드 시 실행 파일 안에 직접 포함되므로 rpath 관리가 불필요하다.

---

## 7. Step 5 — CMakeLists.txt 수정

### 7-1. 최상위 CMakeLists.txt 전체 변경 내용

#### 리눅스 전용 링커 플래그 제거

```cmake
# 기존 (Linux 전용 — macOS ld64에서 에러 발생)
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-no-as-needed")

# 변경 후
if(NOT APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-no-as-needed")
endif()
```

#### GLFW find_package 변경

```cmake
# 기존
find_library(GLFW libglfw.so.3 HINTS ${CMAKE_SOURCE_DIR}/mujoco-3.6.0/bin)

# 변경 후
find_package(glfw3 REQUIRED HINTS /opt/homebrew/lib/cmake/glfw3)
```

#### X11 플랫폼 분기

```cmake
# 기존 (항상 X11 탐색)
find_package(X11)

# 변경 후 (Linux에서만)
if(NOT APPLE)
    find_package(X11 REQUIRED)
endif()
```

#### include_directories — RBDL 경로 추가

```cmake
# 추가된 항목
${CMAKE_SOURCE_DIR}/include/rbdl
/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3
```

#### MuJoCo 라이브러리 플랫폼 분기

```cmake
if(APPLE)
    # macOS: .dylib 사용
    set(MUJOCO_LIB ${MUJOCO_HOME}/lib/libmujoco.dylib)
    set(MUJOCO_ELASTICITY_PLUGIN ${MUJOCO_HOME}/bin/mujoco_plugin/libelasticity.dylib)

    # macOS 전용 Objective-C++ 소스 추가
    list(APPEND COM_SRC
        ${MUJOCO_HOME}/simulate/glfw_corevideo.mm
        ${MUJOCO_HOME}/simulate/macos_gui.mm
    )
    enable_language(OBJCXX)
    set(CMAKE_OBJCXX_STANDARD 17)
else()
    # Linux: .so 사용
    set(MUJOCO_LIB
        ${MUJOCO_HOME}/lib/libmujoco.so
        ${MUJOCO_HOME}/lib/libmujoco.so.3.6.0
    )
    find_package(X11 REQUIRED)
endif()
```

#### target_link_libraries 플랫폼 분기

```cmake
target_link_libraries(run
    control
    ${COM_LIB}      # MUJOCO_LIB + glfw
    ${CMAKE_DL_LIBS}
    OpenGL::GL
    GLEW::glew
    ${OPENGL_LIBRARIES}
    policy
    observation
)

# Linux 전용 추가 링크
if(NOT APPLE)
    target_link_libraries(run
        ${X11_LIBRARIES}
        OpenGL::EGL        # macOS에는 EGL 없음
    )
endif()

# macOS 전용 프레임워크 링크 + rpath 설정
if(APPLE)
    target_link_libraries(run
        "-framework CoreVideo"   # CVDisplayLink (vsync)
        "-framework Cocoa"       # NSApplication, NSWindow
        "-framework IOKit"       # 하드웨어 접근
    )
    set_target_properties(run PROPERTIES
        BUILD_RPATH   "${MUJOCO_HOME}/lib;${CMAKE_SOURCE_DIR}/onnxruntime/lib"
        INSTALL_RPATH "${MUJOCO_HOME}/lib;${CMAKE_SOURCE_DIR}/onnxruntime/lib"
    )
endif()
```

### 7-2. src/robot/CMakeLists.txt

```cmake
# 기존
target_include_directories(robot PUBLIC
    /usr/local/include      # Linux 시스템 경로
)
target_link_libraries(robot PUBLIC
    Eigen3::Eigen
    -lrbdl                  # 시스템 설치된 rbdl 사용
    -lrbdl_urdfreader
)

# 변경 후
target_include_directories(robot PUBLIC
    ${CMAKE_SOURCE_DIR}/include/rbdl                        # 프로젝트 로컬 헤더
    /opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3       # Eigen 3.x 명시
)
target_link_libraries(robot PUBLIC
    Eigen3::Eigen
    ${CMAKE_SOURCE_DIR}/lib/librbdl.a           # 절대 경로로 정적 라이브러리 지정
    ${CMAKE_SOURCE_DIR}/lib/librbdl_urdfreader.a
)
```

### 7-3. src/control/CMakeLists.txt

```cmake
# 기존
target_include_directories(control PUBLIC
    /usr/local/include/rbdl   # Linux 시스템 경로
)

# 변경 후
target_include_directories(control PUBLIC
    ${CMAKE_SOURCE_DIR}/include/rbdl
    /opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3
)
```

### 7-4. src/policy/CMakeLists.txt

```cmake
# 기존 (target_link_directories 를 include에 잘못 사용)
target_link_directories(policy PUBLIC
    ${CMAKE_SOURCE_DIR}/include          # 잘못됨: link가 아닌 include 경로
    ${CMAKE_SOURCE_DIR}/onnxruntime/include
)

# 변경 후 (역할 명확히 분리)
target_include_directories(policy PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/onnxruntime/include
)
target_link_directories(policy PUBLIC
    ${CMAKE_SOURCE_DIR}/onnxruntime/lib
)
```

---

## 8. Step 6 — 코드 버그 수정

빌드 과정에서 기존 코드의 버그 2개가 발견되어 수정했다.

### 버그 1: 타이포 (`dobule` → `double`)

**파일**: `include/controller.h:56`

```cpp
// 기존 (컴파일 에러)
void setVelocityCommand(double vx, dobule vy, double wz){

// 수정 후
void setVelocityCommand(double vx, double vy, double wz){
```

### 버그 2: 잘못된 타입 (`mjModel*` → `const mjModel*`, `mjData*`)

**파일**: `include/controller.h:47-50`

```cpp
// 기존 (mjModel* 를 mjData* 위치에 전달 — 타입 에러)
void setMujocoModel(mjModel* m, mjModel* d){
    Model.set_mujoco_model(m, d);   // d가 mjModel*인데 mjData* 자리에 들어감
    _obs.setMujocoModel(m, d);
}

// 수정 후
void setMujocoModel(const mjModel* m, mjData* d){
    Model.set_mujoco_model(m, d);
    _obs.setMujocoModel(m, d);
}
```

`robotmodel.h`와 `observation.h`의 함수 시그니처를 보면:
```cpp
// robotmodel.h
void set_mujoco_model(const mjModel* m, mjData* d);   // d는 mjData*

// observation.h
void setMujocoModel(const mjModel* m, mjData* d);     // d는 mjData*
```

두 번째 인자는 모두 `mjData*` 이므로 `controller.h`의 타입도 맞춰야 한다.

---

## 9. Step 7 — dylib install name 수정

### 문제

MuJoCo DMG에서 가져온 dylib는 Apple Framework 구조로 패키징되어 있어,  
install name이 framework 내부 경로를 가리킨다:

```bash
otool -D mujoco-3.6.0/lib/libmujoco.3.6.0.dylib
# → @rpath/mujoco.framework/Versions/A/libmujoco.3.6.0.dylib
```

이 경로로는 프로젝트의 rpath(`mujoco-3.6.0/lib/`)에서 찾을 수 없다.  
`mujoco-3.6.0/lib/mujoco.framework/Versions/A/libmujoco.3.6.0.dylib` 라는  
디렉토리가 없기 때문이다.

### 해결: install name 단순화

```bash
# 메인 dylib의 install name 변경
install_name_tool -id \
    "@rpath/libmujoco.3.6.0.dylib" \
    mujoco-3.6.0/lib/libmujoco.3.6.0.dylib

# 코드 서명 무효화되므로 ad-hoc 서명으로 재서명
codesign --force --sign - mujoco-3.6.0/lib/libmujoco.3.6.0.dylib
```

### 플러그인 dylib도 동일하게 처리

플러그인들(`libsdf_plugin.dylib`, `libelasticity.dylib` 등)도  
`libmujoco` 를 framework 경로로 참조하고 있으므로 일괄 수정:

```bash
for f in mujoco-3.6.0/bin/mujoco_plugin/*.dylib; do
    # 참조 경로 변경
    install_name_tool -change \
        "@rpath/mujoco.framework/Versions/A/libmujoco.3.6.0.dylib" \
        "@rpath/libmujoco.3.6.0.dylib" \
        "$f"
    # ad-hoc 재서명
    codesign --force --sign - "$f"
done
```

### 최종 rpath 확인

```bash
otool -l build/run | grep -A3 LC_RPATH
# → /Users/.../mujoco-3.6.0/lib
# → /Users/.../onnxruntime/lib
# → /opt/homebrew/lib

otool -L build/run
# → @rpath/libmujoco.3.6.0.dylib       ← 정상
# → @rpath/libonnxruntime.1.24.4.dylib  ← 정상
# → /opt/homebrew/opt/glfw/lib/libglfw.3.dylib
```

---

## 10. 최종 디렉토리 구조

```
MuJoCo-kist/
├── CMakeLists.txt                      ← 수정됨 (플랫폼 분기)
│
├── mujoco-3.6.0/
│   ├── lib/
│   │   ├── libmujoco.3.6.0.dylib      ← macOS universal2 (신규)
│   │   ├── libmujoco.dylib            ← symlink (신규)
│   │   ├── libmujoco.so               ← Linux용 (유지)
│   │   └── libmujoco.so.3.6.0         ← Linux용 (유지)
│   ├── bin/mujoco_plugin/
│   │   ├── libelasticity.dylib        ← macOS (신규, install name 수정됨)
│   │   ├── libelasticity.so           ← Linux (유지)
│   │   └── ... (기타 플러그인)
│   ├── simulate/
│   │   ├── glfw_corevideo.h           ← macOS 전용 (신규)
│   │   ├── glfw_corevideo.mm          ← macOS 전용 (신규)
│   │   ├── macos_gui.mm               ← macOS 전용 (신규)
│   │   └── ... (기존 파일들)
│   └── include/mujoco/                ← 헤더 업데이트됨
│
├── onnxruntime/                        ← macOS arm64 1.24.4 (교체됨)
│   ├── lib/
│   │   ├── libonnxruntime.dylib
│   │   └── libonnxruntime.1.24.4.dylib
│   └── include/
│
├── onnxruntime-linux-x64/              ← 기존 Linux 버전 (백업)
│
├── lib/                                ← 신규 디렉토리
│   ├── librbdl.a                       ← RBDL 정적 라이브러리 (소스 빌드)
│   └── librbdl_urdfreader.a            ← RBDL URDF 파서 (소스 빌드)
│
├── include/
│   ├── rbdl/                           ← 신규: RBDL 헤더
│   └── ... (기존 헤더들)
│
└── src/
    ├── control/CMakeLists.txt          ← 수정됨
    ├── robot/CMakeLists.txt            ← 수정됨
    ├── policy/CMakeLists.txt           ← 수정됨
    └── ...
```

---

## 11. 빌드 방법

### 요구 사항 (Homebrew)

```bash
brew install glfw glew eigen
# eigen 3.x 버전이 설치되어 있는지 확인
ls /opt/homebrew/Cellar/eigen/3.4.0_1
```

### 빌드 명령

```bash
mkdir build && cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DEigen3_DIR=/opt/homebrew/Cellar/eigen/3.4.0_1/share/eigen3/cmake

make -j$(sysctl -n hw.ncpu)

./run
```

#### 옵션 설명

| 옵션 | 의미 |
|------|------|
| `-DCMAKE_BUILD_TYPE=Release` | 최적화 빌드 (`-O2`). 디버깅 필요 시 `Debug`로 변경 |
| `-DEigen3_DIR=...` | Eigen 3.4.0 경로 명시. 생략 시 Eigen 5.x 가 잡혀 RBDL 빌드 실패 |
| `-j$(sysctl -n hw.ncpu)` | CPU 코어 수만큼 병렬 컴파일 (빌드 속도 향상) |

### 빌드 결과 확인

```bash
# 아키텍처 확인
file build/run
# → Mach-O 64-bit executable arm64

# 링크된 라이브러리 확인
otool -L build/run

# 정상 실행 확인
./build/run
# → [INFO]  FILE PATH  :  ../pyfile/exported/policy.onnx
# → [INFO]  INPUT DIM  :  820
# → [INFO]  Output DIM :  31
# → MuJoCo version 3.6.0
```

---

## 12. 트러블슈팅

### 문제 1: `GLFW::glfw` target not found

```
Target "run" links to: GLFW::glfw but the target was not found.
```

**원인**: Homebrew GLFW의 cmake target 이름이 `glfw` (namespace 없음).  
**해결**:
```cmake
# 잘못됨
find_package(GLFW3 REQUIRED)
target_link_libraries(run GLFW::glfw)

# 올바름
find_package(glfw3 REQUIRED HINTS /opt/homebrew/lib/cmake/glfw3)
target_link_libraries(run glfw)
```

### 문제 2: GLEW_LIBRARY NOTFOUND

```
GLEW_LIBRARY is set to NOTFOUND.
```

**원인**: GLEW cmake 타겟 이름이 `GLEW::GLEW` 가 아닌 `GLEW::glew`.  
**해결**:
```cmake
# 잘못됨
target_link_libraries(run GLEW::GLEW ${GLEW_LIBRARIES})

# 올바름
target_link_libraries(run GLEW::glew)
```

### 문제 3: Eigen `_check_template_params` 에러

```
error: no member named '_check_template_params'
```

**원인**: RBDL이 Eigen 3.x API를 사용하는데 Eigen 5.x가 선택됨.  
**해결**: cmake 실행 시 Eigen 3.x 경로 명시:
```bash
cmake .. -DEigen3_DIR=/opt/homebrew/Cellar/eigen/3.4.0_1/share/eigen3/cmake
```

### 문제 4: dylib 찾을 수 없음 (plugin 로딩 실패)

```
Error loading plugin library: Library not loaded: @rpath/mujoco.framework/...
```

**원인**: 플러그인 dylib들이 framework 구조의 경로로 libmujoco를 참조함.  
**해결**: [Step 7](#9-step-7--dylib-install-name-수정) 참고 — `install_name_tool`로 경로 수정 후 재서명.

### 문제 5: `-Wl,-no-as-needed` 링커 에러

```
ld: unknown option: -no-as-needed
```

**원인**: macOS의 링커 `ld64`는 이 옵션을 지원하지 않음 (Linux `ld`전용).  
**해결**:
```cmake
if(NOT APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-no-as-needed")
endif()
```

### 문제 6: `OpenGL::EGL` not found

```
Target "OpenGL::EGL" not found
```

**원인**: EGL은 Linux/Android용. macOS는 CGL/Metal 사용.  
**해결**:
```cmake
if(NOT APPLE)
    target_link_libraries(run OpenGL::EGL)
endif()
```
