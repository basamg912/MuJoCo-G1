#include "robotmodel.h"
#define RJDOF 17
// #define RFJDOF 7+6

CModelKAPEX::CModelKAPEX()
{
	Initialize();
}

CModelKAPEX::~CModelKAPEX()
{
}

void CModelKAPEX::Initialize()
{
	m_bool_model_update = false;
	m_bool_kinematics_update = false;
	m_bool_dynamics_update = false;
	m_bool_Jacobian_update = false;

	m_DoF = RJDOF;
	m_q.setZero(RJDOF);
	m_qdot.setZero(RJDOF);

	m_zero_vec_joint.setZero();

	m_id_left_heel = 6;
	m_id_left_toe = 7;

	m_A.setZero();
	m_g.setZero();
	m_b.setZero();
	m_bg.setZero();

	m_A_tmp.setZero(RJDOF, RJDOF);
	m_g_tmp.setZero(RJDOF);
	m_b_tmp.setZero(RJDOF);
	m_bg_tmp.setZero(RJDOF);

	m_J_tmp.setZero(6,m_DoF);

	m_J_left_heel.setZero();	
	m_x_left_heel.setZero();					
	m_R_left_heel.setZero();					
	m_xdot_left_heel.setZero();				
	
	m_J_left_toe.setZero();	
	m_x_left_toe.setZero();					
	m_R_left_toe.setZero();					
	m_xdot_left_toe.setZero();				

	set_robot_config();
	load_model();
}

void CModelKAPEX::update_modelID()
{
}

void CModelKAPEX::load_model()
{
	//read urdf model
	RigidBodyDynamics::Addons::URDFReadFromFile("../model/KAPEX(250724)/urdf/KAPEX.urdf", &_model, false, false);	// floating 선택, figure 선택	true false

	cout <<"Model Loaded for RBDL." << endl << "Total DoFs: " << _model.dof_count << endl;
	if (_model.dof_count != m_DoF)//
	{
		cout << "Simulation model and RBDL model mismatch!!!" << endl;
	}

	m_bool_model_update = true; //check model update

	cout << "Model Loading Complete." << endl;
}

void CModelKAPEX::update_kinematics(Vector<double, NUM_OF_JOINTS> & q, Vector<double, NUM_OF_JOINTS> & qdot)
{
	m_q = q.head(RJDOF);
	m_qdot = qdot.head(RJDOF);
	
	if (m_bool_model_update == true)
	{
		RigidBodyDynamics::UpdateKinematicsCustom(_model, &m_q, &m_qdot, NULL); //update kinematics
	}
	else
	{
		cout << "Robot model is not ready. Please load model first." << endl;
	}
	m_bool_kinematics_update = true; //check kinematics update
}

void CModelKAPEX::update_dynamics()
{
	if (m_bool_kinematics_update == true)
	{
		RigidBodyDynamics::CompositeRigidBodyAlgorithm(_model, m_q, m_A_tmp, false); //update dynamics
		RigidBodyDynamics::InverseDynamics(_model, m_q, m_zero_vec_joint, m_zero_vec_joint, m_g_tmp, NULL); //get m_g
		RigidBodyDynamics::InverseDynamics(_model, m_q, m_qdot, m_zero_vec_joint, m_bg_tmp, NULL); //get m_g+m_b
		m_b_tmp = m_bg_tmp - m_g_tmp; //get m_b

		m_A = m_A_tmp.block<RJDOF,RJDOF>(0,0);
		m_g = m_g_tmp.segment<RJDOF>(0);
		m_b = m_b_tmp.segment<RJDOF>(0);
		m_bg = m_bg_tmp.segment<RJDOF>(0);
	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl;
	}
	m_bool_dynamics_update = true; //check kinematics update
}
 
void CModelKAPEX::calculate_EE_Jacobians()
{
	if (m_bool_kinematics_update == true)
	{		
		m_J_tmp.setZero();	m_J_left_heel.setZero();
		RigidBodyDynamics::CalcPointJacobian6D(_model, m_q, m_id_left_heel, m_position_local_task_left_heel, m_J_tmp, false); //left heel		
		m_J_left_heel.block<3, RJDOF>(0, 0) =  m_J_tmp.block<3, RJDOF>(3, 0);
		m_J_left_heel.block<3, RJDOF>(3, 0) =  m_J_tmp.block<3, RJDOF>(0, 0);
		// cout<<"m_J_left_heel"<<endl<<m_J_tmp<<endl;
		
		m_J_tmp.setZero();	m_J_left_toe.setZero();
		RigidBodyDynamics::CalcPointJacobian6D(_model, m_q, m_id_left_toe, m_position_local_task_left_toe, m_J_tmp, false); //left toe		
		m_J_left_toe.block<3, RJDOF>(0, 0) =  m_J_tmp.block<3, RJDOF>(3, 0);
		m_J_left_toe.block<3, RJDOF>(3, 0) =  m_J_tmp.block<3, RJDOF>(0, 0);
		// cout<<"m_J_left_toe"<<endl<<m_J_tmp<<endl;

		m_bool_Jacobian_update = true;
	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl;
	}
}
 
void CModelKAPEX::calculate_EE_positions_orientations()
{
	if (m_bool_kinematics_update == true)
	{
		m_x_left_heel.setZero();
		m_x_left_heel = RigidBodyDynamics::CalcBodyToBaseCoordinates(_model, m_q, m_id_left_heel, m_position_local_task_left_heel, false);

		m_x_left_toe.setZero();
		m_x_left_toe = RigidBodyDynamics::CalcBodyToBaseCoordinates(_model, m_q, m_id_left_toe, m_position_local_task_left_toe, false);

		m_R_left_heel.setZero();
		m_R_left_heel = (RigidBodyDynamics::CalcBodyWorldOrientation(_model, m_q, m_id_left_heel, false).transpose());

		m_R_left_toe.setZero();
		m_R_left_toe = (RigidBodyDynamics::CalcBodyWorldOrientation(_model, m_q, m_id_left_toe, false).transpose());
	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl;
	}
}

void CModelKAPEX::calculate_EE_velocity()
{
	if (m_bool_Jacobian_update == true)
	{

		m_xdot_left_heel = m_J_left_heel * m_qdot;
		m_xdot_left_toe = m_J_left_toe * m_qdot;

		// cout<<"m_xdot_left_heel :"<<m_xdot_left_heel.transpose()<<endl;
		// cout<<"m_xdot_left_toe :"<<m_xdot_left_toe.transpose()<<endl;
	}
	else
	{
		cout << "Jacobian matrices are not ready. Please calculate Jacobians first." << endl;
	}
}

void CModelKAPEX::set_robot_config()
{
	m_position_local_task_left_heel.setZero();
	m_position_local_task_left_heel << 0.03219874, 0.0, 0.08583056;
	m_position_local_task_left_toe.setZero();
	m_position_local_task_left_toe << 0.021, 0.0265021, 0.0;
}