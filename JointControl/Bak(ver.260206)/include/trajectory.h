#pragma once
#ifndef __TRAJECTORY_H
#define __TRAJECTORY_H

#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace Eigen;
using namespace std;

class CTrajectory{
	public:
		CTrajectory();
		virtual ~CTrajectory();
			
		int _vector_size;
		double _time_start, _time, _time_end;	
		VectorXd _init_pos, _init_vel, _goal_pos, _goal_vel;	
		void set_size(int dof);
		void reset_initial(double time0, VectorXd init_pos, VectorXd init_vel);
		void update_time(double time);
		void update_goal(VectorXd goal_pos, VectorXd goal_vel, double goal_time);
		int check_trajectory_complete();
		VectorXd position_cubicSpline();
		VectorXd velocity_cubicSpline();
		Matrix3d rotationCubic();
		Vector3d rotationCubicDot();
		MatrixXd _init_rot, _goal_rot;
		void reset_initial_rotation(double time_0, const Matrix3d &rotation_0);
		void update_goal_rotation(double goal_time,const Matrix3d &rotation_f);

	private:
		void Initialize();
		void check_vector_size(VectorXd X);
		bool _bool_trajectory_complete;
};

#endif