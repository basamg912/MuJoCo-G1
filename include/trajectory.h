#pragma once
#ifndef __TRAJECTORY_H
#define __TRAJECTORY_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "custommath.h"

using namespace std;
using namespace Eigen;

class CTrajectory
{

public:
	CTrajectory();
	virtual ~CTrajectory();

	void set_size(int dof);
	void reset_initial(double time0, VectorXd init_pos, VectorXd init_vel);
	void update_time(double time);
	void update_goal(VectorXd goal_pos, VectorXd goal_vel, double goal_time);
	int check_trajectory_complete();

	VectorXd position_cubicSpline();
	VectorXd velocity_cubicSpline();

private:
	void Initialize();
	void check_vector_size(VectorXd X);

	int _vector_size;
	double _time_start, _time, _time_end;
	VectorXd _init_pos, _init_vel, _goal_pos, _goal_vel;

	bool _bool_trajectory_complete;
	double _motion_threshold;
};

#endif
