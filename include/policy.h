#pragma once
#ifndef _POLICY_H
#define _POLICY_H

#include <onnxruntime_cxx_api.h>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>

class Policy
{
public:
    Policy(const std::string& model_path);
    ~Policy();

    Eigen::VectorXd inference(const Eigen::VectorXd& obs); // ! input : 164dim , history length 5 = 820 dim

    // ? 상수 멤버 함수 -> 멤버 변수 수정 불가능한 함수
    int getInputDim() const {return _input_dim;}
    int getOutputDim() const {return _output_dim; }

private:
    int _input_dim;
    int _output_dim;
    std::string _input_name;
    std::string _output_name;

    // ? {nullptr} -> 객체 생성자 호출 후 초기화
    // Ort : onnxruntime 
    Ort::Env _env; // ! onnx runtime 관리 객체
    Ort::Session _session{nullptr}; // ! onnx 파일 로드, 실행
    Ort::MemoryInfo _memory_info{nullptr};
};

#endif