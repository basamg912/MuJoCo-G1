#pragma once
#ifndef __MODEL_H
#define __MODEL_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <mujoco/mujoco.h>
#include "custommath.h"

using namespace std;
using namespace Eigen;

class CModel
{
public:
	CModel();
	virtual ~CModel();

    int get_qpos_offset();
    int get_qvel_offset();

    void set_mujoco_model(const mjModel* m, mjData* d);
    void update_kinematics(VectorXd & q, VectorXd & qdot);
    void update_dynamics();
    const mjModel* getMjModel(){
        return _mj_model;
    }

    MatrixXd _A;   // inertia matrix
    VectorXd _g;   // gravity force vector
	VectorXd _b;   // Coriolis/centrifugal force vector
	VectorXd _bg;  // Coriolis/centrifugal + gravity

private:
	void Initialize();

    VectorXd _q, _qdot;
    VectorXd _zero_vec_joint;

    int _qvel_offset;
    int _qpos_offset;
    int _k;

    const mjModel* _mj_model;
    mjData* _mj_data;

    bool _bool_model_update, _bool_kinematics_update, _bool_dynamics_update;
};

#endif
