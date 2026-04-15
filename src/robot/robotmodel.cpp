#include "robotmodel.h"
#define JDOF 31



CModel::CModel()
{
	Initialize();
}

CModel::~CModel()
{
}

void CModel::Initialize()
{
	_bool_model_update = false;
	_bool_kinematics_update = false;
	_bool_dynamics_update = false;
	_bool_Jacobian_update = false;

    _k = JDOF;
    _id_hand = 7;

	_max_joint_torque.setZero(_k);
	_min_joint_torque.setZero(_k);
	_max_joint_velocity.setZero(_k);
	_min_joint_velocity.setZero(_k);
	_max_joint_position.setZero(_k);
	_min_joint_position.setZero(_k);

    _q.setZero(_k);
    _qdot.setZero(_k);
    _zero_vec_joint.setZero(_k);

    _A.setZero(_k,_k);
    _g.setZero(_k);
	_b.setZero(_k);
	_bg.setZero(_k);

    _J_hand.setZero(6,_k);
    _J_tmp.setZero(6,_k);

    _position_local_task_hand.setZero(); // 3x1
	_tmp_position_local_task_hand.setZero(); // 3x1
	// _tmp_position_local_task_hand(0) = 0.088;

    _x_hand.setZero(); // 3x1
    _R_hand.setZero(); // 3x3

    _xdot_hand.setZero(6);

	set_robot_config();
	// ! 4/7 RBDL -> Mujoco
    // load_model();
	_mj_model = nullptr;
	_mj_data = nullptr;
}

// ! 4/7 RBDL -> Mujoco
// void CModel::load_model()
// {   
//     // RigidBodyDynamics::Addons::URDFReadFromFile("../model/panda_arm_hand.urdf", &_model, false, true);
// 	RigidBodyDynamics::Addons::URDFReadFromFile("../model/kapex/KAPEX_wo_hand_head.urdf", &_model, false, false);

//     cout << endl << endl << "Model Loaded for RBDL." << endl << "Total DoFs: " << _model.dof_count << endl << endl;
// 	if (_model.dof_count != _k)
// 	{
// 		cout << "Simulation model and RBDL model mismatch!!!" << endl << endl;
// 	}

//     _bool_model_update = true; //check model update

// 	cout << "Model Loading Complete." << endl << endl;

// }

int CModel::get_qpos_offset(){
	return _qpos_offset;
}
int CModel::get_qvel_offset(){
	return _qvel_offset;
}

void CModel::set_mujoco_model(const mjModel* m, mjData* d){
	_mj_model = m;
	_mj_data = d;

	_qpos_offset = 0;
	_qvel_offset = 0;
	for (int i=0; i<_mj_model->njnt; i++){
		if (_mj_model->jnt_type[i] == mjJNT_FREE){
			_qpos_offset += 7;
			_qvel_offset += 6;
		}
	}

	_bool_model_update = true;
}

void CModel::update_kinematics(VectorXd & q, VectorXd & qdot)
{
	_q = q;
	_qdot = qdot;
	_bool_kinematics_update = true; // check kinematics update
}

void CModel::update_dynamics()
{
    if (_bool_kinematics_update == true)
    {
        // 질량행렬
		int nv = _mj_model->nv; // ! dof 갯수
		mjtNum M[nv*nv];
        mj_fullM(_mj_model, M, _mj_data->qM);
        for (int i = 0; i < _k; i++)
            for (int j = 0; j < _k; j++)
                _A(i, j) = M[ (i+ _qvel_offset) * nv + (j+ _qvel_offset)];

        // Coriolis + gravity
        for (int i = 0; i < _k; i++)
            _bg(i) = _mj_data->qfrc_bias[i+ _qvel_offset];
    }
    _bool_dynamics_update = true;
}

void CModel::calculate_EE_Jacobians()
{
    if (_bool_kinematics_update == true)
    {
        _J_hand.setZero();
        mjtNum jacp[3 * _mj_model->nv];
        mjtNum jacr[3 * _mj_model->nv];
        mj_jacBody(_mj_model, _mj_data, jacp, jacr, _hand_body_id);

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < _k; j++) {
                _J_hand(i, j)     = jacp[i * _mj_model->nv + j]; // linear
                _J_hand(i + 3, j) = jacr[i * _mj_model->nv + j]; // angular
            }
        _bool_Jacobian_update = true;
    }
}

void CModel::calculate_EE_positions_orientations()
{
    if (_bool_kinematics_update == true)
    {
        for (int i = 0; i < 3; i++)
            _x_hand(i) = _mj_data->xpos[3 * _hand_body_id + i];

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                _R_hand(i, j) = _mj_data->xmat[9 * _hand_body_id + i * 3 + j];
    }
}

void CModel::calculate_EE_velocity()
{
	if (_bool_Jacobian_update == true)
	{
		_xdot_hand = _J_hand * _qdot;
	}
	else
	{
		cout << "Jacobian matrices are not ready. Please calculate Jacobians first." << endl << endl;
	}
}

void CModel::set_robot_config(){
	
	_min_joint_position(0) = -2.8973;
	_min_joint_position(1) = -1.7628;
	_min_joint_position(2) = -2.8973;
	_min_joint_position(3) = -3.0718;
	_min_joint_position(4) = -2.8973;
	_min_joint_position(5) = -0.0175;
	_min_joint_position(6) = -2.8973;

	_max_joint_position(0) = 2.8973;
	_max_joint_position(1) = 1.7628;
	_max_joint_position(2) = 2.8973;
	_max_joint_position(3) = -0.0698;
	_max_joint_position(4) = 2.8973;
	_max_joint_position(5) = 3.7525;
	_max_joint_position(6) = 2.8973;

	_max_joint_velocity(0) = 2.175;
	_max_joint_velocity(1) = 2.175;
	_max_joint_velocity(2) = 2.175;
	_max_joint_velocity(3) = 2.175;
	_max_joint_velocity(4) = 2.61;
	_max_joint_velocity(5) = 2.61;
	_max_joint_velocity(6) = 2.61;
	_min_joint_velocity = -_max_joint_velocity;

}
