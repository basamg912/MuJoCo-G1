#include "controller.h"
#include "mapping.h"
#include <algorithm>
#include <cmath>

CController::CController() {
  _k = 29; // G1 29-DOF
  Initialize();
}

CController::~CController() {}

void CController::set_default_pose(mjData *d) {
  int offset = Model.get_qpos_offset();

  for (int i = 0; i < Model.getMjModel()->nq; i++) {
    d->qpos[i] = 0;
  }
  for (int i = 0; i < Model.getMjModel()->nv; i++) {
    d->qvel[i] = 0;
  }
  for (int i = 0; i < Model.getMjModel()->na; i++) {
    d->qacc[i] = 0;
  }

  d->qpos[0] = 0.0;
  d->qpos[1] = 0.0;
  d->qpos[2] = 0.8;
  d->qpos[3] = 1.0; // ! wxyz 순서
  d->qpos[4] = 0.0;
  d->qpos[5] = 0.0;
  d->qpos[6] = 0.0;

  for (int i = 0; i < 29; i++) {
    d->qpos[_qpos_adr[i]] = g1_default_pose[i];
  }
}

void CController::read(double t, double *q, double *qdot) {
  _t = t;
  if (_bool_init == true) {
    _init_t = _t;
    _bool_init = false;
  }

  _dt = t - _pre_t;
  _pre_t = t;

  for (int i = 0; i < _k; i++) {
    _q(i) = q[i + Model.get_qpos_offset()];
    _qdot(i) = qdot[i + Model.get_qvel_offset()];
    _pre_q(i) = _q(i);
    _pre_qdot(i) = _qdot(i);
  }
}

void CController::step_pd() { JointControl(); }

void CController::write(double *ctrl) {
  for (int i = 0; i < _k; i++) {
    double lim = g1_effort_limit[i];
    ctrl[i] = std::max(-lim, std::min(lim, _torque(i)));
  }
}

void CController::control_mujoco() {
  ModelUpdate();
  Eigen::VectorXd stacked_obs = _obs.update(_q, _qdot, _q_home, _last_action);
  static int cnt = 0;
  // if (cnt++ < 1) {
  //   std::cout << "[INFO qhome] " << _q_home(0) << '\n';
  //   std::cout << "[INFO dof pos] " << (_q - _q_home).transpose() << '\n';
  //   std::cout << "[obs] " << stacked_obs.transpose() << "\n";
  // }
  Eigen::VectorXd action = _policy->inference(stacked_obs);

  // robot.py: clip_actions=True, action_clip_value=100.0
  action = action.cwiseMax(-100.0).cwiseMin(100.0);

  _last_action = action;

  // robot.py: action_scale=0.25
  _q_des = _q_home + 0.25 * action.head(_k);

  // robot.py: dof_pos_lower/upper_limit_list — q_des 를 하드웨어 한계 내로 클램핑
  for (int i = 0; i < _k; i++) {
    _q_des(i) = std::max(g1_pos_lower_limit[i], std::min(g1_pos_upper_limit[i], _q_des(i)));
  }

  _qdot_des.setZero();
  JointControl();
}

void CController::ModelUpdate() {
  Model.update_kinematics(_q, _qdot);
  Model.update_dynamics();
}

void CController::JointControl() {
  _torque = _kp_diag.cwiseProduct(_q_des - _q) +
            _kd_diag.cwiseProduct(_qdot_des - _qdot);
}

void CController::reset() {
  _obs.reset();
  _last_action.setZero(_last_action.size());
  _bool_init = true;
  _pre_t = 0.0;
  _t = 0.0;
  _init_t = 0.0;
  _dt = 0.0;
  _q_des = _q_home;
  _qdot_des.setZero(_k);
}

void CController::Initialize() {
  _bool_init = true;
  _t = 0.0;
  _init_t = 0.0;
  _pre_t = 0.0;
  _dt = 0.0;

  _q.setZero(_k);
  _qdot.setZero(_k);
  _torque.setZero(_k);
  _pre_q.setZero(_k);
  _pre_qdot.setZero(_k);

  _q_home.resize(_k);
  for (int i = 0; i < _k; i++)
    _q_home(i) = g1_default_pose[i];
  _q_des = _q_home;
  _qdot_des.setZero(_k);

  _kp_diag.setZero(_k);
  _kd_diag.setZero(_k);

  // G1 29dof actuator 순서 기준 (학습 robot.py 값과 정확히 일치)
  // LL (0~5): left_hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
  _kp_diag(0) = 40.179238471;
  _kd_diag(0) = 2.557889765; // left_hip_pitch
  _kp_diag(1) = 99.098427777;
  _kd_diag(1) = 6.308801854; // left_hip_roll
  _kp_diag(2) = 40.179238471;
  _kd_diag(2) = 2.557889765; // left_hip_yaw
  _kp_diag(3) = 99.098427777;
  _kd_diag(3) = 6.308801854; // left_knee
  _kp_diag(4) = 28.501246196;
  _kd_diag(4) = 1.814445687; // left_ankle_pitch
  _kp_diag(5) = 28.501246196;
  _kd_diag(5) = 1.814445687; // left_ankle_roll

  // RL (6~11): right_hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch,
  // ankle_roll
  _kp_diag(6) = 40.179238471;
  _kd_diag(6) = 2.557889765; // right_hip_pitch
  _kp_diag(7) = 99.098427777;
  _kd_diag(7) = 6.308801854; // right_hip_roll
  _kp_diag(8) = 40.179238471;
  _kd_diag(8) = 2.557889765; // right_hip_yaw
  _kp_diag(9) = 99.098427777;
  _kd_diag(9) = 6.308801854; // right_knee
  _kp_diag(10) = 28.501246196;
  _kd_diag(10) = 1.814445687; // right_ankle_pitch
  _kp_diag(11) = 28.501246196;
  _kd_diag(11) = 1.814445687; // right_ankle_roll

  // Waist (12~14): yaw, roll, pitch
  _kp_diag(12) = 40.179238471;
  _kd_diag(12) = 2.557889765; // waist_yaw
  _kp_diag(13) = 28.501246196;
  _kd_diag(13) = 1.814445687; // waist_roll
  _kp_diag(14) = 28.501246196;
  _kd_diag(14) = 1.814445687; // waist_pitch

  // LA (15~21): left_shoulder_pitch, roll, yaw, elbow, wrist_roll, wrist_pitch,
  // wrist_yaw
  _kp_diag(15) = 14.250623098;
  _kd_diag(15) = 0.907222843; // left_shoulder_pitch
  _kp_diag(16) = 14.250623098;
  _kd_diag(16) = 0.907222843; // left_shoulder_roll
  _kp_diag(17) = 14.250623098;
  _kd_diag(17) = 0.907222843; // left_shoulder_yaw
  _kp_diag(18) = 14.250623098;
  _kd_diag(18) = 0.907222843; // left_elbow
  _kp_diag(19) = 14.250623098;
  _kd_diag(19) = 0.907222843; // left_wrist_roll
  _kp_diag(20) = 16.778327481;
  _kd_diag(20) = 1.068141502; // left_wrist_pitch
  _kp_diag(21) = 16.778327481;
  _kd_diag(21) = 1.068141502; // left_wrist_yaw

  // RA (22~28): right_shoulder_pitch, roll, yaw, elbow, wrist_roll,
  // wrist_pitch, wrist_yaw
  _kp_diag(22) = 14.250623098;
  _kd_diag(22) = 0.907222843; // right_shoulder_pitch
  _kp_diag(23) = 14.250623098;
  _kd_diag(23) = 0.907222843; // right_shoulder_roll
  _kp_diag(24) = 14.250623098;
  _kd_diag(24) = 0.907222843; // right_shoulder_yaw
  _kp_diag(25) = 14.250623098;
  _kd_diag(25) = 0.907222843; // right_elbow
  _kp_diag(26) = 14.250623098;
  _kd_diag(26) = 0.907222843; // right_wrist_roll
  _kp_diag(27) = 16.778327481;
  _kd_diag(27) = 1.068141502; // right_wrist_pitch
  _kp_diag(28) = 16.778327481;
  _kd_diag(28) = 1.068141502; // right_wrist_yaw
}
