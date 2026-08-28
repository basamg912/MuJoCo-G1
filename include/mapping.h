#pragma once

struct ActuatorParam{
    double X1;
    double X2;
    double Y1;
    double SF;
    double DF;
};
constexpr ActuatorParam Example= {
    .X1 = 10,
    .X2 = 15,
    .Y1 = 250,
    .SF = 5,
    .DF = 0.5,
};

constexpr const ActuatorParam* JOINT_ACTUATOR_MAP[31]={
    // ===== Example Legs (LL/RL) =====
    /*  0 hip yaw   */ &Example,
    /*  1 hip roll  */ &Example,
    /*  2 hip pitch */ &Example,
    /*  3 knee      */ &Example,
    /*  4 ankle p   */ &Example,
    /*  5 ankle r   */ &Example,
    /*  6 toe       */ &Example,
    &Example,
    &Example,
    &Example,
    &Example,
    &Example,
    &Example,
    &Example,
};
static const int isaac_leg_to_mujoco[14] = {
    0, 7, 1, 8, 2, 9, 3, 10, 4, 11, 5, 12, 6, 13
};
static const int isaac_joint_to_mujoco[31] = {
    0,
    7,
    14,
    1,
    8,
    15,
    2,
    9,
    16,
    3,
    10,
    17,
    24,
    4,
    11,
    18,
    25,
    5,
    12,
    19,
    26,
    6,
    13,
    20,
    27,
    21,
    28,
    22,
    29,
    23,
    30,
};

static const double Example_Leg_kp[14] = {
    50, 50, 150, 150, 30, 30, 5,
    50, 50, 150, 150, 30, 30, 5
};
static const double Example_Leg_kd[14] = {
    10, 10, 20, 20, 2.0, 2.0, 0.05,
    10, 10, 20, 20, 2.0, 2.0, 0.05
};

static const double joint_kp[31] = {
    50, 50, 150, 150, 30, 30, 5,
    50, 50, 150, 150, 30, 30, 5,
    220, 350, 400,
    70, 70, 5, 5, 2, 5, 5,
    70, 70, 5, 5, 2, 5, 5
};

static const double joint_kd[31] = {
    10, 10, 20, 20, 2.0, 2.0, 0.05,
    10, 10, 20, 20, 2.0, 2.0, 0.05,
    25, 50, 50,
    8, 8, 1, 1, 0.5, 0.05, 0.05,
    8, 8, 1, 1, 0.5, 0.05, 0.05
};

// effort limit (MuJoCo 순서)
static const double joint_effort_limit[31] = {
    50, 50, 150, 150, 35, 35, 10,
    50, 50, 150, 150, 35, 35, 10,
    100, 200, 200,
    50, 50, 30, 30, 20, 10, 10,
    50, 50, 30, 30, 20, 10, 10
};

// velocity limit (MuJoCo 순서)
static const double joint_vel_limit[31] = {
    10, 10, 10, 10, 8, 8, 5,
    10, 10, 10, 10, 8, 8, 5,
    15, 15, 15,
    5, 5, 5, 5, 5, 5, 5,
    5, 5, 5, 5, 5, 5, 5
};
