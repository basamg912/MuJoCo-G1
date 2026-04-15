#pragma once
#ifndef __MODEL_H
#define __MODEL_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/rbdl_math.h>
#include "custommath.h"
#include <eigen3/Eigen/Geometry>

#include "StateDef.h"

using namespace std;
using namespace Eigen;
typedef double Scalar;

class CModelKAPEX{
	public:
		CModelKAPEX();
		virtual ~CModelKAPEX();

		void update_kinematics(Vector<double, NUM_OF_JOINTS>& q, Vector<double, NUM_OF_JOINTS>& qdot);
		void update_dynamics();
		void calculate_EE_positions_orientations();
		void calculate_EE_Jacobians();	
		void calculate_EE_velocity();
		void update_modelID();

		RigidBodyDynamics::Model _model;

		Matrix<double, 17, 17> m_A; //inertia matrix
		Vector<double, 17> m_g; //gravity force vector
		Vector<double, 17> m_b; //Coriolis/centrifugal force vector
		Vector<double, 17> m_bg; //Coriolis/centrifugal force vector + gravity force vector

		MatrixXd m_A_tmp;
		VectorXd m_g_tmp, m_b_tmp, m_bg_tmp;

		Matrix<double, 6, 17> m_J_left_heel;
		Vector<double, 3> m_x_left_heel;
		Matrix<double, 3, 3> m_R_left_heel;
		Vector<double, 6> m_xdot_left_heel;

		Matrix<double, 6, 17> m_J_left_toe;
		Vector<double, 3> m_x_left_toe;
		Matrix<double, 3, 3> m_R_left_toe;
		Vector<double, 6> m_xdot_left_toe;

	private:
		void Initialize();
		void load_model();
		void set_robot_config();

		int m_DoF;//joint number

		bool m_bool_model_update, m_bool_kinematics_update, m_bool_dynamics_update, m_bool_Jacobian_update;

		//ankle : from knee
		int m_id_left_heel, m_id_left_toe;

		// VectorCXd _joint_num_rbdl;

		VectorXd m_q, m_qdot;
		Vector<double, 7> m_zero_vec_joint;

		Vector<double, 3> m_position_local_task_left_heel;
		Vector<double, 3> m_position_local_task_left_toe;

		MatrixXd m_J_tmp;
};

#endif