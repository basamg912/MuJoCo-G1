#pragma once
#include <onnxruntime_cxx_api.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>
using namespace std;
class AttentionPolicy{
public:
    AttentionPolicy(const string &model_path,
                    int expected_n_joints = -1,
                    int expected_n_feet = -1);
    Eigen::VectorXd inference(
        const Eigen::MatrixXd &joint_desc,
        const Eigen::MatrixXd &joint_obs,
        const Eigen::MatrixXd &feet_desc,
        const Eigen::MatrixXd &feet_obs,
        const Eigen::MatrixXd &global_obs
    );
    int n_joints() const {return _n_joints;}
    int n_feet() const {return _n_feet;}
    int global_dim() const {return _global_dim;}
private:
    int _n_joints, _n_feet, _global_dim, _joint_desc_dim, _joint_obs_dim, _feet_desc_dim, _feet_obs_dim;
    static constexpr const char* kInputNames[5] = {
        "joint_desc",
        "joint_obs",
        "feet_desc",
        "feet_obs",
        "global_obs",
    };
    static constexpr const char* kOutputNames = "actions";
    Ort::Env _env;
    Ort::Session _session{nullptr};
    Ort::MemoryInfo _memory_info{nullptr};
};
