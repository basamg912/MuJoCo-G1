#include "g1_controller.h"

#include <algorithm>
#include <iostream>
#include <stdexcept>

G1Controller::G1Controller()
{
    _q.resize(G1_N_JOINTS);
    _qdot.resize(G1_N_JOINTS);
    _q_home.resize(G1_N_JOINTS);
    _q_des.resize(G1_N_JOINTS);
    _last_action.resize(G1_N_JOINTS);

    for (int i = 0; i < G1_N_JOINTS; ++i) {
        _q_home(i) = g1_q_home[i];
    }
    _q.setZero();
    _qdot.setZero();
    _q_des = _q_home;
    _last_action.setZero();
}

void G1Controller::setModel(mjModel* m, mjData* d)
{
    _mj_model = m;
    _mj_data = d;
    cacheModelIndices();
    applyArmature();
    _attn_obs.setMujocoModel(m, d);
    _attn_obs.reset();
}

void G1Controller::loadAttentionPolicy(const std::string& onnx_path,
                                       const std::string& joint_desc_bin,
                                       const std::string& feet_desc_bin)
{
    _attn_policy = std::make_unique<AttentionPolicy>(onnx_path,
                                                     G1_N_JOINTS,
                                                     G1_N_FEET);
    _attn_obs.loadStaticDescription(joint_desc_bin, feet_desc_bin);
    _last_action.setZero();
}

void G1Controller::setDefaultPose(mjData* d)
{
    if (_mj_model == nullptr) {
        throw std::runtime_error("[G1Controller] setModel must be called first");
    }

    for (int i = 0; i < _mj_model->nq; ++i) {
        d->qpos[i] = _mj_model->qpos0[i];
    }
    for (int i = 0; i < _mj_model->nv; ++i) {
        d->qvel[i] = 0.0;
    }
    for (int i = 0; i < _mj_model->nu; ++i) {
        d->ctrl[i] = 0.0;
    }

    int floating_joint = mj_name2id(_mj_model, mjOBJ_JOINT, "floating_base_joint");
    if (floating_joint >= 0 && _mj_model->jnt_type[floating_joint] == mjJNT_FREE) {
        int adr = _mj_model->jnt_qposadr[floating_joint];
        d->qpos[adr + 0] = 0.0;
        d->qpos[adr + 1] = 0.0;
        d->qpos[adr + 2] = 0.8;
        d->qpos[adr + 3] = 1.0;
        d->qpos[adr + 4] = 0.0;
        d->qpos[adr + 5] = 0.0;
        d->qpos[adr + 6] = 0.0;
    }

    for (int i = 0; i < G1_N_JOINTS; ++i) {
        d->qpos[_qpos_addr[i]] = _q_home(i);
    }

    _q = _q_home;
    _qdot.setZero();
    _q_des = _q_home;
    _last_action.setZero();
    _last_policy_time = -1.0;

    mj_forward(_mj_model, d);
}

void G1Controller::reset()
{
    _t = 0.0;
    _last_policy_time = -1.0;
    _printed_first_policy = false;
    _q_des = _q_home;
    _last_action.setZero();
}

void G1Controller::read(double time, const double* qpos, const double* qvel)
{
    _t = time;
    for (int i = 0; i < G1_N_JOINTS; ++i) {
        _q(i) = qpos[_qpos_addr[i]];
        _qdot(i) = qvel[_dof_addr[i]];
    }
}

void G1Controller::controlMujoco()
{
    if (!_attn_policy) {
        _q_des = _q_home;
        return;
    }

    if (_last_policy_time < 0.0 || _t - _last_policy_time >= 0.02) {
        _last_policy_time = _t;

        Eigen::MatrixXd joint_obs = _attn_obs.computeJointObs(_q, _qdot, _q_home, _last_action);
        Eigen::MatrixXd feet_obs = _attn_obs.computeFeetObs();
        Eigen::VectorXd global_obs = _attn_obs.computeGlobalObs();

        Eigen::VectorXd action = _attn_policy->inference(
            _attn_obs.jointDesc(),
            joint_obs,
            _attn_obs.feetDesc(),
            feet_obs,
            global_obs
        );

        if (!_printed_first_policy) {
            std::cout << "[G1] global_obs: " << global_obs.transpose() << '\n';
            std::cout << "[G1] feet_obs:\n" << feet_obs << '\n';
            std::cout << "[G1] action: " << action.transpose() << '\n';
            _printed_first_policy = true;
        }

        Eigen::VectorXd action_mj(G1_N_JOINTS);
        action_mj.setZero();
        for (int isaac_idx = 0; isaac_idx < G1_N_JOINTS; ++isaac_idx) {
            int mj_idx = g1_isaac_joint_to_mujoco[isaac_idx];
            action_mj(mj_idx) = action(isaac_idx);
        }

        _last_action = action;
        _q_des = _q_home + _action_scale * action_mj;
    }
}

void G1Controller::write(double* ctrl)
{
    for (int mj_idx = 0; mj_idx < G1_N_JOINTS; ++mj_idx) {
        double qdot_clamped = std::clamp(
            _qdot(mj_idx),
            -g1_joint_vel_limit[mj_idx],
            g1_joint_vel_limit[mj_idx]
        );
        double torque = g1_joint_kp[mj_idx] * (_q_des(mj_idx) - _q(mj_idx))
                      - g1_joint_kd[mj_idx] * qdot_clamped;
        torque = std::clamp(
            torque,
            -g1_joint_effort_limit[mj_idx],
            g1_joint_effort_limit[mj_idx]
        );

        int actuator_id = _actuator_ids[mj_idx];
        if (actuator_id >= 0) {
            ctrl[actuator_id] = torque;
        }
    }
}

void G1Controller::setVelocityCommand(double vx, double vy, double wz)
{
    _attn_obs.setVelocityCommand(vx, vy, wz);
}

void G1Controller::setComCommand(double dx, double dy, double dz)
{
    _attn_obs.setComCommand(dx, dy, dz);
}

void G1Controller::cacheModelIndices()
{
    for (int i = 0; i < G1_N_JOINTS; ++i) {
        const char* name = g1_mujoco_joint_names[i];
        int joint_id = mj_name2id(_mj_model, mjOBJ_JOINT, name);
        if (joint_id < 0) {
            throw std::runtime_error(std::string("[G1Controller] joint not found: ") + name);
        }
        _joint_ids[i] = joint_id;
        _qpos_addr[i] = _mj_model->jnt_qposadr[joint_id];
        _dof_addr[i] = _mj_model->jnt_dofadr[joint_id];

        int actuator_id = mj_name2id(_mj_model, mjOBJ_ACTUATOR, name);
        if (actuator_id < 0) {
            throw std::runtime_error(std::string("[G1Controller] actuator not found: ") + name);
        }
        _actuator_ids[i] = actuator_id;
    }

    std::cout << "[G1Controller] cached " << G1_N_JOINTS << " joints/actuators\n";
}

void G1Controller::applyArmature()
{
    for (int i = 0; i < G1_N_JOINTS; ++i) {
        _mj_model->dof_armature[_dof_addr[i]] = g1_joint_armature[i];
    }
}
