#include "observation.h"
#include "mapping.h"
#include <iostream>

Observation::Observation()
    : _mj_model(nullptr), _mj_data(nullptr), _pelvis_body_id(-1),
      _phase(0.0), _phase_freq(1.0) // 학습: gait_period=1.0s → freq=1.0Hz
{
    _vel_cmd.setZero(); // ! 초기 속도 명령은 0 으로 부여
}

Observation::~Observation(){}

void Observation::setMujocoModel(const mjModel* m, mjData* d){
    _mj_model = m;
    _mj_data = d;

    _pelvis_body_id = mj_name2id(m, mjOBJ_BODY, "pelvis");
    if(_pelvis_body_id < 0) std::cout << "[INFO] Pelvis body not found" << '\n';
    std::cout << "[INFO] Pelvis ID : " << _pelvis_body_id << '\n';
}

void Observation::setVelocityCommand(double vx, double vy, double wz){
    _vel_cmd << vx, vy, wz;
}

void Observation::reset(){
    _history.clear();
    _phase = 0.0;
}

Eigen::VectorXd Observation::computeSingleObs(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& qdot,
    const Eigen::VectorXd& q_home,
    const Eigen::VectorXd& last_action
)
{
    Eigen::VectorXd obs(SINGLE_DIM);
    int idx = 0;

    // 공통 계산: base_quat_inv (world->body)
    mjtNum base_quat[4] = {
        _mj_data->xquat[4*_pelvis_body_id + 0],
        _mj_data->xquat[4*_pelvis_body_id + 1],
        _mj_data->xquat[4*_pelvis_body_id + 2],
        _mj_data->xquat[4*_pelvis_body_id + 3],
    };
    mjtNum base_quat_inv[4];
    mju_negQuat(base_quat_inv, base_quat);

    // stand 판별
    double cmd_norm = std::sqrt(_vel_cmd(0)*_vel_cmd(0) + _vel_cmd(1)*_vel_cmd(1));
    stand = (cmd_norm < 0.1) && (std::fabs(_vel_cmd(2)) < 0.1);

    // phase 업데이트
    _phase = std::fmod(2.0 * M_PI * _phase_freq * _mj_data->time, 2.0 * M_PI);

    // ── 알파벳 순 (학습 코드 sorted(term_names) 와 동일) ──────────────────

    // 1) actions (29) × 1.0
    for (int i = 0; i < DOF; i++) obs(idx++) = last_action(i);

    // 2) base_ang_vel (3) × 0.25
    // qvel[3:6] for free joint = world-frame angular velocity; rotate to body frame
    mjtNum ang_vel_world[3] = {_mj_data->qvel[3], _mj_data->qvel[4], _mj_data->qvel[5]};
    mjtNum ang_vel_body[3];
    mju_rotVecQuat(ang_vel_body, ang_vel_world, base_quat_inv);
    for (int i = 0; i < 3; i++) obs(idx++) = ang_vel_body[i] * 0.25;

    // 3) command_ang_vel (1) × 1.0
    obs(idx++) = _vel_cmd(2);

    // 4) command_lin_vel (2) × 1.0
    obs(idx++) = _vel_cmd(0);
    obs(idx++) = _vel_cmd(1);

    // 5) cos_phase (2) × 1.0
    if (stand) {
        obs(idx++) = std::cos(M_PI);   // -1
        obs(idx++) = std::cos(M_PI);   // -1
    } else {
        obs(idx++) = std::cos(_phase);
        obs(idx++) = std::cos(_phase + M_PI);
    }

    // 6) dof_pos (29) × 1.0
    for (int i = 0; i < DOF; i++) obs(idx++) = q(i) - q_home(i);

    // 7) dof_vel (29) × 0.05
    for (int i = 0; i < DOF; i++) obs(idx++) = qdot(i) * 0.05;

    // 8) projected_gravity (3) × 1.0
    mjtNum gravity_world[3] = {0.0, 0.0, -1.0};
    mjtNum gravity_body[3];
    mju_rotVecQuat(gravity_body, gravity_world, base_quat_inv);
    for (int i = 0; i < 3; i++) obs(idx++) = gravity_body[i];

    // 9) sin_phase (2) × 1.0
    if (stand) {
        obs(idx++) = std::sin(M_PI);   // 0
        obs(idx++) = std::sin(M_PI);   // 0
    } else {
        obs(idx++) = std::sin(_phase);
        obs(idx++) = std::sin(_phase + M_PI);
    }

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
    // std::cout << "[INFO - obs] " <<single_obs;
    
    if (_history.empty()) {
        for (int i = 0; i < HISTORY_LEN; i++)
            _history.push_back(single_obs);
    } else {
        _history.push_back(single_obs);
        if ((int)_history.size() > HISTORY_LEN) _history.pop_front();
    }

    Eigen::VectorXd stacked(STACKED_DIM);
    int idx = 0;
    for (const auto& obs : _history) {
        stacked.segment(idx, SINGLE_DIM) = obs;
        idx += SINGLE_DIM;
    }
    return stacked;
}
