#include <iostream>
#include <stdexcept>
#include "attention_policy.h"
#include "onnxruntime_c_api.h"
#include "onnxruntime_cxx_api.h"
using namespace std;
AttentionPolicy::AttentionPolicy(const string &model_path,
                                 int expected_n_joints,
                                 int expected_n_feet)
: _env(ORT_LOGGING_LEVEL_WARNING, "attention_policy")
{
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(1);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    _session = Ort::Session(_env, model_path.c_str(), opts);
    _memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    size_t num_inputs = _session.GetInputCount();
    size_t num_outputs = _session.GetOutputCount();
    if (num_inputs != 5){
        throw runtime_error("Expected 5 inputs, got " + to_string(num_inputs));
    }
    if (num_outputs != 1){
        throw runtime_error("Expected 1 output, got " + to_string(num_outputs));
    }
    Ort::AllocatorWithDefaultOptions allocator;
    for (size_t i=0; i<num_inputs; i++){
        auto name_ptr = _session.GetInputNameAllocated(i, allocator);
        if (string(name_ptr.get()) != kInputNames[i]){
            throw runtime_error("Expected input " + to_string(i) + " to be " + kInputNames[i] + ", got " + string(name_ptr.get()));
        }
    }
    auto shape_of = [&](size_t i) {
        return _session.GetInputTypeInfo(i)
            .GetTensorTypeAndShapeInfo()
            .GetShape();
    };
    auto sh_jd = shape_of(0);
    auto sh_jo = shape_of(1);
    auto sh_fd = shape_of(2);
    auto sh_fo = shape_of(3);
    auto sh_go = shape_of(4);
    for (size_t i=0; i<sh_go.size(); i++){
        cout << "[INFO] global ob dim : " << sh_go[i] << (i+1 < sh_go.size() ? " , " : "")<<'\n';
    }
    if (sh_go.size() < 2)   throw runtime_error("global obs rank < 2");
    _global_dim = static_cast<int>(sh_go.back());
    if (_global_dim != 15){
        throw runtime_error("global obs dim mismath");
    }

    // Resolve a dim that may be dynamic (reported as -1 by ORT). Prefer the
    // static side of (descriptor, obs); fall back to the caller's expected
    // size when both are dynamic. Inconsistent statics still fail loudly.
    auto resolve_dim = [](const char* name, int desc_dim, int obs_dim,
                          int expected) {
        if (desc_dim > 0 && obs_dim > 0 && desc_dim != obs_dim) {
            throw runtime_error(
                string("Inconsistent ") + name + " count between desc (" +
                to_string(desc_dim) + ") and obs (" + to_string(obs_dim) + ")");
        }
        int resolved = desc_dim > 0 ? desc_dim : obs_dim;
        if (resolved <= 0) {
            if (expected <= 0) {
                throw runtime_error(
                    string(name) + " count is dynamic on both desc and obs "
                    "inputs and no expected count was provided");
            }
            return expected;
        }
        if (expected > 0 && expected != resolved) {
            throw runtime_error(
                string(name) + " count from ONNX (" + to_string(resolved) +
                ") disagrees with expected (" + to_string(expected) + ")");
        }
        return resolved;
    };

    _n_joints = resolve_dim("joints",
                            static_cast<int>(sh_jd[1]),
                            static_cast<int>(sh_jo[1]),
                            expected_n_joints);
    _joint_desc_dim = static_cast<int>(sh_jd[2]);
    _joint_obs_dim = static_cast<int>(sh_jo[2]);
    _n_feet = resolve_dim("feet",
                          static_cast<int>(sh_fd[1]),
                          static_cast<int>(sh_fo[1]),
                          expected_n_feet);
    _feet_desc_dim = static_cast<int>(sh_fd[2]);
    _feet_obs_dim = static_cast<int>(sh_fo[2]);

    auto out_name_ptr = _session.GetOutputNameAllocated(0, allocator);
    if (string(out_name_ptr.get()) != kOutputNames){
        throw runtime_error(
            string("[AttentionPolicy] output name expected to be ") + kOutputNames + ", got " + string(out_name_ptr.get())
        );
    }
    auto out_shape = _session.GetOutputTypeInfo(0)
        .GetTensorTypeAndShapeInfo()
        .GetShape();
    int out_n_joints = static_cast<int>(out_shape[1]);
    if (out_n_joints > 0 && out_n_joints != _n_joints){
        throw runtime_error("[AttentionPolicy] output shape expected to be " + to_string(_n_joints) + " joints, got " + to_string(out_n_joints));
    }
    cout<< "[INFO] Attention Policy Loaded : " << model_path << '\n'
        << "[INFO] n_joints : " << _n_joints << '\n'
        << "[INFO] n_feet : " << _n_feet << '\n'
        << "[INFO] global_dim : " << _global_dim << '\n'
        << "[INFO] joint_desc_dim : " << _joint_desc_dim << '\n'
        << "[INFO] joint_obs_dim : " << _joint_obs_dim << '\n'
        << "[INFO] feet_desc_dim : " << _feet_desc_dim << '\n'
        << "[INFO] feet_obs_dim : " << _feet_obs_dim << '\n';
}
Eigen::VectorXd AttentionPolicy::inference(
    const Eigen::MatrixXd &joint_desc,
    const Eigen::MatrixXd &joint_obs,
    const Eigen::MatrixXd &feet_desc,
    const Eigen::MatrixXd &feet_obs,
    const Eigen::MatrixXd &global_obs
)
{
    auto check = [](const char* name, const Eigen::MatrixXd& m, int rows, int cols) {
        if (m.rows() != rows || m.cols() != cols) {
            throw runtime_error(
                string(name) + " shape mismatch: policy expects " +
                to_string(rows) + "x" + to_string(cols) + ", got " +
                to_string(m.rows()) + "x" + to_string(m.cols()));
        }
    };
    check("joint_desc", joint_desc, _n_joints, _joint_desc_dim);
    check("joint_obs",  joint_obs,  _n_joints, _joint_obs_dim);
    check("feet_desc",  feet_desc,  _n_feet,   _feet_desc_dim);
    check("feet_obs",   feet_obs,   _n_feet,   _feet_obs_dim);
    if (global_obs.size() != _global_dim) {
        throw runtime_error(
            "global_obs size mismatch: policy expects " +
            to_string(_global_dim) + ", got " + to_string(global_obs.size()));
    }

    vector<float> jd_buf(joint_desc.size());
    vector<float> jo_buf(joint_obs.size());
    vector<float> fd_buf(feet_desc.size());
    vector<float> fo_buf(feet_obs.size());
    vector<float> go_buf(global_obs.size());

    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        jd_buf.data(), joint_desc.rows(), joint_desc.cols()
    ) = joint_desc.cast<float>();
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        jo_buf.data(), joint_obs.rows(), joint_obs.cols()
    ) = joint_obs.cast<float>();
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        fd_buf.data(), feet_desc.rows(), feet_desc.cols()
    ) = feet_desc.cast<float>();
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        fo_buf.data(), feet_obs.rows(), feet_obs.cols()
    ) = feet_obs.cast<float>();
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        go_buf.data(), global_obs.rows(), global_obs.cols()
    ) = global_obs.cast<float>();
    vector<int64_t> jd_shape = {1, _n_joints, _joint_desc_dim};
    vector<int64_t> jo_shape = {1, _n_joints, _joint_obs_dim};
    vector<int64_t> fd_shape = {1, _n_feet,   _feet_desc_dim};
    vector<int64_t> fo_shape = {1, _n_feet,   _feet_obs_dim};
    vector<int64_t> go_shape = {1, _global_dim};
    vector<Ort::Value> inputs;
    inputs.push_back(Ort::Value::CreateTensor<float>(_memory_info, jd_buf.data(), jd_buf.size(), jd_shape.data(), jd_shape.size()));
    inputs.push_back(Ort::Value::CreateTensor<float>(_memory_info, jo_buf.data(), jo_buf.size(), jo_shape.data(), jo_shape.size()));
    inputs.push_back(Ort::Value::CreateTensor<float>(_memory_info, fd_buf.data(), fd_buf.size(), fd_shape.data(), fd_shape.size()));
    inputs.push_back(Ort::Value::CreateTensor<float>(_memory_info, fo_buf.data(), fo_buf.size(), fo_shape.data(), fo_shape.size()));
    inputs.push_back(Ort::Value::CreateTensor<float>(_memory_info, go_buf.data(), go_buf.size(), go_shape.data(), go_shape.size()));

    const char* input_names[] = {"joint_desc", "joint_obs", "feet_desc", "feet_obs", "global_obs"};
    const char* output_names[] = {"actions"};

    auto outputs = _session.Run(
        Ort::RunOptions{nullptr},
        input_names, inputs.data(), inputs.size(),
        output_names, 1
    );

    float* out_data = outputs[0].GetTensorMutableData<float>();
    Eigen::VectorXd actions(_n_joints);
    for (int i=0; i<_n_joints; i++){
        actions[i] = out_data[i];
    }
    return actions;

}
