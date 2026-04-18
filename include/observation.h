#pragma once
#ifndef _OBSERVATION_H
#define _OBSERVATION_H

#include <eigen3/Eigen/Dense>
#include <deque>
#include <cmath>
#include <mujoco/mujoco.h>

class Observation
{
public:
    Observation();
    ~Observation();
    bool stand;
    void setMujocoModel(const mjModel* m, mjData* d);
    void setVelocityCommand(double vx, double vy, double wz);
    Eigen::VectorXd update(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& qdot,
        const Eigen::VectorXd& q_home,
        const Eigen::VectorXd& last_action
    );
    void reset();

private:
    const mjModel* _mj_model;
    mjData* _mj_data;
    int _pelvis_body_id;

    Eigen::VectorXd computeSingleObs(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& qdot,
        const Eigen::VectorXd& q_home,
        const Eigen::VectorXd& last_action
    );

    // 3(ang_vel) + 3(gravity) + 2(lin_cmd) + 1(ang_cmd)
    // + 29(dof_pos) + 29(dof_vel) + 29(actions)
    // + 2(sin_phase: left,right) + 2(cos_phase: left,right) = 100
    static constexpr int DOF         = 29;
    static constexpr int HISTORY_LEN = 1;
    static constexpr int SINGLE_DIM  = 100;
    static constexpr int STACKED_DIM = SINGLE_DIM * HISTORY_LEN; // 100

    std::deque<Eigen::VectorXd> _history;
    Eigen::Vector3d _vel_cmd;  // vx, vy, wz

    double _phase;       // 현재 위상 [0, 2π)
    double _phase_freq;  // 보행 주파수 [Hz]
};

#endif
