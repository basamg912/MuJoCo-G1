#pragma once

#include "attention_policy.h"
#include "g1_attention_observation.h"
#include "g1_mapping.h"

#include <eigen3/Eigen/Dense>
#include <mujoco/mujoco.h>

#include <array>
#include <memory>
#include <string>

class G1Controller
{
public:
    G1Controller();

    void setModel(mjModel* m, mjData* d);
    void loadAttentionPolicy(const std::string& onnx_path,
                             const std::string& joint_desc_bin,
                             const std::string& feet_desc_bin);
    void setDefaultPose(mjData* d);
    void reset();

    void read(double time, const double* qpos, const double* qvel);
    void controlMujoco();
    void write(double* ctrl);

    void setVelocityCommand(double vx, double vy, double wz);
    void setComCommand(double dx, double dy, double dz);

    const Eigen::VectorXd& q() const { return _q; }
    const Eigen::VectorXd& qDes() const { return _q_des; }
    const Eigen::VectorXd& lastAction() const { return _last_action; }

private:
    void cacheModelIndices();
    void applyArmature();

    mjModel* _mj_model = nullptr;
    mjData* _mj_data = nullptr;
    std::array<int, G1_N_JOINTS> _joint_ids{};
    std::array<int, G1_N_JOINTS> _qpos_addr{};
    std::array<int, G1_N_JOINTS> _dof_addr{};
    std::array<int, G1_N_JOINTS> _actuator_ids{};

    double _t = 0.0;
    double _last_policy_time = -1.0;
    double _action_scale = 0.25;
    bool _printed_first_policy = false;

    Eigen::VectorXd _q;
    Eigen::VectorXd _qdot;
    Eigen::VectorXd _q_home;
    Eigen::VectorXd _q_des;
    Eigen::VectorXd _last_action;

    G1AttentionObservation _attn_obs;
    std::unique_ptr<AttentionPolicy> _attn_policy;
};
