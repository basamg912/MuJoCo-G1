// robot.py dof_pos_lower_limit_list (dof_names 순서 기준)
const double g1_pos_lower_limit[29] = {
    -2.5307, -0.5236, -2.7576, -0.087267, -0.87267, -0.2618,          // left leg
    -2.5307, -2.9671, -2.7576, -0.087267, -0.87267, -0.2618,          // right leg
    -2.618,  -0.52,   -0.52,                                            // waist
    -3.0892, -1.5882, -2.618, -1.0472, -1.972222054, -1.61443, -1.61443, // left arm
    -3.0892, -2.2515, -2.618, -1.0472, -1.972222054, -1.61443, -1.61443, // right arm
};

// robot.py dof_pos_upper_limit_list
const double g1_pos_upper_limit[29] = {
    2.8798, 2.9671, 2.7576, 2.8798, 0.5236, 0.2618,                   // left leg
    2.8798, 0.5236, 2.7576, 2.8798, 0.5236, 0.2618,                   // right leg
    2.618,  0.52,   0.52,                                               // waist
    2.6704, 2.2515, 2.618, 2.0944, 1.972222054, 1.61443, 1.61443,     // left arm
    2.6704, 1.5882, 2.618, 2.0944, 1.972222054, 1.61443, 1.61443,     // right arm
};

// robot.py dof_vel_limit_list
const double g1_vel_limit[29] = {
    32.0, 20.0, 32.0, 20.0, 37.0, 37.0,  // left leg
    32.0, 20.0, 32.0, 20.0, 37.0, 37.0,  // right leg
    32.0, 37.0, 37.0,                      // waist
    37.0, 37.0, 37.0, 37.0, 37.0, 22.0, 22.0, // left arm
    37.0, 37.0, 37.0, 37.0, 37.0, 22.0, 22.0, // right arm
};

const double g1_default_pose[29]={
    -0.312, 0.0, 0.0, 0.669, -0.363, 0.0, // left leg:  pitch, roll, yaw, knee, ankle_pitch, ankle_roll
    -0.312, 0.0, 0.0, 0.669, -0.363, 0.0, // right leg
    0.0, 0.0, 0.0,                         // waist: yaw, roll, pitch
    0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0,    // left arm:  s_pitch, s_roll, s_yaw, elbow, w_roll, w_pitch, w_yaw
    0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0,   // right arm
};

// robot.py dof_effort_limit_list
const double g1_effort_limit[29] = {
    88.0, 139.0, 88.0, 139.0, 50.0, 50.0,  // left leg
    88.0, 139.0, 88.0, 139.0, 50.0, 50.0,  // right leg
    88.0,  50.0, 50.0,                       // waist
    25.0,  25.0, 25.0, 25.0, 25.0, 5.0, 5.0, // left arm
    25.0,  25.0, 25.0, 25.0, 25.0, 5.0, 5.0, // right arm
};

// 학습 환경(Isaac Lab)의 flip_sign_joint_names 에 해당하는 관절: 1 = 부호 반전
const int g1_flip_sign[29] = {
    0, 1, 1, 0, 0, 1,       // left leg:  pitch, roll*, yaw*, knee, ankle_pitch, ankle_roll*
    0, 1, 1, 0, 0, 1,       // right leg
    1, 1, 0,                 // waist: yaw*, roll*, pitch
    0, 1, 1, 0, 1, 0, 1,    // left arm:  s_pitch, s_roll*, s_yaw*, elbow, w_roll*, w_pitch, w_yaw*
    0, 1, 1, 0, 1, 0, 1,    // right arm
};

