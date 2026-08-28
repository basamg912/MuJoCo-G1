#include "observation.h"
#include <cmath>
#include <iostream>
#include "mapping.h"

Observation::Observation()
    : _mj_model(nullptr), _mj_data(nullptr), _pelvis_body_id(-1)
{
    _vel_cmd.setZero();
    // ? Eigen::Vector3d method overload
    _default_com << -0.066071861, -0.19773669, 0.0024721903;
}

Observation::~Observation(){}

void Observation::setMujocoModel(const mjModel* m, mjData* d){
    _mj_model = m;
    _mj_data = d;

    _pelvis_body_id = mj_name2id(m, mjOBJ_BODY, "pelvis");
    if(_pelvis_body_id <0)  std::cout << "[INFO] Pelvis body not found" <<'\n';
    std::cout << "[INFO] Body num : " << m->nbody << '\n';
    std::cout<<"[INFO] Pelvis ID : " << _pelvis_body_id << '\n';
    // for (int i=0; i< m->nbody; i++){
    //     const char* name = mj_id2name(m, mjOBJ_BODY, i);
    //     std::cout << "[INFO] body id "<<"'" << i<< "'" << " name : " << name <<'\n';
    // }
    // for (int i=0; i< m->njnt; i++){
    //     const char* name = mj_id2name(m, mjOBJ_JOINT, i);
    //     std::cout << "[INFO] joint id "<<"'" << i<< "'" << " name : " << name <<'\n';
    // }

}

void Observation::setComCommand(double dx, double dy, double dz){
    _com_cmd << dx, dy, dz;
}
void Observation::setVelocityCommand(double vx, double vy, double wz){
    _vel_cmd << vx, vy, wz;
}

void Observation::reset(){
    _history.clear(); // ? deque clear
}

// ! this is for legacy policy - local pc RL result test
Eigen::VectorXd Observation::computeSingleObs_temp(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& qdot,
    const Eigen::VectorXd& q_home,
    const Eigen::VectorXd& last_action
)
{
    Eigen::VectorXd obs(SINGLE_DIM);
    int idx=0;

    // base quat conjugate: world -> body
    mjtNum base_quat[4] = {
        _mj_data->xquat[4*_pelvis_body_id +0],
        _mj_data->xquat[4*_pelvis_body_id +1],
        _mj_data->xquat[4*_pelvis_body_id +2],
        _mj_data->xquat[4*_pelvis_body_id +3],
    };
    mjtNum base_quat_inv[4];
    mju_negQuat(base_quat_inv, base_quat);

    // * 1) base_lin_vel (body frame, no scale)
    // mjtNum lin_vel_world[3] = {
    //     _mj_data->qvel[0], _mj_data->qvel[1], _mj_data->qvel[2]
    // };
    // mjtNum lin_vel_body[3];
    // mju_rotVecQuat(lin_vel_body, lin_vel_world, base_quat_inv);
    // for (int i=0; i<3; i++) obs(idx++) = lin_vel_body[i];

    // * 2) base_ang_vel (body frame, no scale)
    mjtNum ang_vel_world[3] = {
        _mj_data->qvel[3], _mj_data->qvel[4], _mj_data->qvel[5]
    };
    mjtNum ang_vel_body[3];
    mju_rotVecQuat(ang_vel_body, ang_vel_world, base_quat_inv);
    for (int i=0; i<3; i++) obs(idx++) = ang_vel_body[i]*0.2;

    // * 3) projected_gravity
    mjtNum gravity_world[3] = {0.0, 0.0, -1.0};
    mjtNum gravity_body[3];
    mju_rotVecQuat(gravity_body, gravity_world, base_quat_inv);
    for (int i=0; i<3; i++) obs(idx++) = gravity_body[i];

    // * 4) joint_pos_rel
    for (int i=0; i<JOINT_DIM; i++) obs(idx++) = q(isaac_joint_to_mujoco[i]) - q_home(isaac_joint_to_mujoco[i]);

    // * 5) joint_vel_rel (no scale)
    for (int i=0; i<JOINT_DIM; i++) obs(idx++) = qdot(isaac_joint_to_mujoco[i])*0.05;

    // * 6) last_action
    for (int i=0; i<last_action.size(); i++) obs(idx++) = last_action(i);

    return obs;
}

// ! this is for spark RL result test
Eigen::VectorXd Observation::computeSingleObs(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& qdot,
    const Eigen::VectorXd& q_home,
    const Eigen::VectorXd& last_action
)
{
    // ! layout: base_lin_vel(3)+ang_vel(3)+gravity(3)+vel_cmd(3)+joint_pos(31)+joint_vel(31)+last_action(31)+dif_torso_com(3) = 108
    Eigen::VectorXd obs(SINGLE_DIM);
    int idx=0;

    // ! base quaternion + conjugate (world -> body 변환용)
    mjtNum base_quat[4] = {
        _mj_data->xquat[4*_pelvis_body_id +0],
        _mj_data->xquat[4*_pelvis_body_id +1],
        _mj_data->xquat[4*_pelvis_body_id +2],
        _mj_data->xquat[4*_pelvis_body_id +3],
    };
    mjtNum base_quat_inv[4];
    mju_negQuat(base_quat_inv, base_quat);

    // * 1) base_lin_vel (body frame, isaaclab root_lin_vel_b 와 동일)
    // mjtNum lin_vel_world[3] = {
    //     _mj_data->qvel[0], _mj_data->qvel[1], _mj_data->qvel[2]
    // };
    // mjtNum lin_vel_body[3];
    // mju_rotVecQuat(lin_vel_body, lin_vel_world, base_quat_inv);
    // for (int i=0; i<3; i++) obs(idx++) = lin_vel_body[i];

    // * 2) base_ang_vel (scale 0.2)
    // ! MuJoCo free joint 의 qvel[3:6] 은 이미 body-local angular velocity 다.
    //   base_quat_inv 로 다시 돌리면 '이중 변환' 이 되어 IsaacLab root_ang_vel_b 와 불일치한다.
    //   standing(ang_vel≈0)에선 영향이 없지만, 전진/회전 시 이중 변환된 ang_vel 이
    //   obs 를 학습분포 밖(OOD)으로 밀어 정책 action 이 발산한다. → raw 그대로 사용.
    for (int i = 0; i < 3; i++) obs(idx++) = _mj_data->qvel[3 + i] * 0.2;

    // * 2) gravity
    // ? 월드좌표상으로는 -z 방향 -> 로봇의 기울어진 방향 기준으로 변경. 중력의 힘보다 방향을 중요시 -> norm 을 1로
    mjtNum gravity_world[3] = {0.0, 0.0, -1.0};
    mjtNum gravity_body[3];
    mju_rotVecQuat(gravity_body, gravity_world, base_quat_inv);
    for(int i=0; i<3; i++)  obs(idx++) = gravity_body[i];

    // * 3) vel
    for (int i=0; i<3; i++) obs[idx++] = _vel_cmd[i];

    // ! leg-only 정책(14 DOF): 다리 관절만 obs 에 포함 (isaac leg order → mujoco)
    // * 4) joint_pos_rel (다리 14)
    for (int i=0; i<LEG_DIM; i++)   obs[idx++]= q(isaac_leg_to_mujoco[i]) - q_home(isaac_leg_to_mujoco[i]);

    // * 5) joint_vel_rel (다리 14) x0.05
    for (int i=0; i<LEG_DIM; i++)   obs[idx++] = qdot(isaac_leg_to_mujoco[i]) * 0.05;

    // * 6) last_action
    for (int i=0; i<last_action.size(); i++)    obs[idx++] = last_action(i);

    // * 7) torso_com dif
    // int wl3_id = mj_name2id(_mj_model, mjOBJ_BODY, "WL3");
    // for(int i=0; i<3; i++){
    //     obs(idx++) = _com_cmd[i];
    // }

    // * 7) gait phase (L/R sin·cos), command 있을 때만 활성 (|vel_cmd|>0.1). Isaac gait_phase 대응.
    {
        double t = _mj_data->time;
        const double period = 0.8;
        double phase   = std::fmod(t, period) / period;
        double phase_l = phase;
        double phase_r = std::fmod(phase + 0.5, 1.0);
        const double two_pi = 2.0 * M_PI;
        double mask = (_vel_cmd.norm() > 0.1) ? 1.0 : 0.0;
        obs[idx++] = std::sin(two_pi * phase_l) * mask;
        obs[idx++] = std::cos(two_pi * phase_l) * mask;
        obs[idx++] = std::sin(two_pi * phase_r) * mask;
        obs[idx++] = std::cos(two_pi * phase_r) * mask;
    }
        // ! debug 용, first observation
    // static bool once = true;
    // if (once) {
    //     std::cout << "ang_vel: " << obs.segment(0, 3).transpose() << std::endl;
    //     std::cout << "gravity: " << obs.segment(3, 3).transpose() << std::endl;
    //     std::cout << "joint_pos: " << obs.segment(9, 14).transpose() << std::endl;
    //     std::cout << "torso_com: " << obs.segment(68, 3).transpose() << std::endl;
    //     once = false;
    // }

    return obs;
}

Eigen::VectorXd Observation::update(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& qdot,
    const Eigen::VectorXd& q_home,
    const Eigen::VectorXd& last_action
)
{

    Eigen::VectorXd single_obs = computeSingleObs(q, qdot, q_home, last_action);

    if(_history.empty()){
        for (int i=0; i<HISTORY_LEN; i++){
            _history.push_back(single_obs);
        }
    }
    else{
        _history.push_back(single_obs);
        if ( (int)_history.size() > HISTORY_LEN ) _history.pop_front();
    }
    // HISTORY_LEN = 5
    //
    Eigen::VectorXd stacked(STACKED_DIM);
    int idx=0;
    // ! single obs layout: base_lin_vel(3)+ang_vel(3)+gravity(3)+vel_cmd(3)+joint_pos(31)+joint_vel(31)+last_action(31)+com_cmd(3) = 108
    // 3 + 3 + 3 + 31 + 31 + 31
    // leg-only single obs(55): angvel3, grav3, velcmd3, jpos14, jvel14, last_action14, gait4
    int term_starts[] = {0, 3, 6, 9, 23, 37, 51};
    int term_size[]   = {3,3,3,14,14,14,4};
    int num_terms = 7;
    for (int i=0; i<num_terms; i++){
        for (int j=0; j< HISTORY_LEN ; j++){
            stacked.segment(idx, term_size[i]) = _history[j].segment(term_starts[i], term_size[i]);
            idx += term_size[i];
        }
    }
    return stacked;
}
