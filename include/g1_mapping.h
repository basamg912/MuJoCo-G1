#pragma once

static constexpr int G1_N_JOINTS = 23;
static constexpr int G1_N_FEET = 2;

// ONNX/IsaacLab joint order -> MuJoCo actuator/joint order.
static const int g1_isaac_joint_to_mujoco[G1_N_JOINTS] = {
    0, 6, 12,
    1, 7, 13, 18,
    2, 8, 14, 19,
    3, 9, 15, 20,
    4, 10, 16, 21,
    5, 11, 17, 22,
};

static const char* const g1_mujoco_joint_names[G1_N_JOINTS] = {
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
};

// IsaacLab init_state, reordered to MuJoCo joint order.
static const double g1_q_home[G1_N_JOINTS] = {
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
    0.0,
    0.3, 0.25, 0.0, 0.97, 0.15,
    0.3, -0.25, 0.0, 0.97, -0.15,
};

// IsaacLab actuator fields, reordered to MuJoCo joint order.
static const double g1_joint_kp[G1_N_JOINTS] = {
    100.0, 100.0, 100.0, 150.0, 40.0, 40.0,
    100.0, 100.0, 100.0, 150.0, 40.0, 40.0,
    200.0,
    40.0, 40.0, 40.0, 40.0, 40.0,
    40.0, 40.0, 40.0, 40.0, 40.0,
};

static const double g1_joint_kd[G1_N_JOINTS] = {
    2.0, 2.0, 2.0, 4.0, 2.0, 2.0,
    2.0, 2.0, 2.0, 4.0, 2.0, 2.0,
    5.0,
    1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0,
};

static const double g1_joint_effort_limit[G1_N_JOINTS] = {
    88.0, 139.0, 88.0, 139.0, 35.0, 35.0,
    88.0, 139.0, 88.0, 139.0, 35.0, 35.0,
    88.0,
    25.0, 25.0, 25.0, 25.0, 25.0,
    25.0, 25.0, 25.0, 25.0, 25.0,
};

static const double g1_joint_vel_limit[G1_N_JOINTS] = {
    32.0, 20.0, 32.0, 20.0, 30.0, 30.0,
    32.0, 20.0, 32.0, 20.0, 30.0, 30.0,
    32.0,
    37.0, 37.0, 37.0, 37.0, 37.0,
    37.0, 37.0, 37.0, 37.0, 37.0,
};

static const double g1_joint_armature[G1_N_JOINTS] = {
    0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
    0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
    0.01,
    0.01, 0.01, 0.01, 0.01, 0.01,
    0.01, 0.01, 0.01, 0.01, 0.01,
};
