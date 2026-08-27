#include "robotmodel.h"
#define JDOF 29

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

    _k = JDOF;

    _q.setZero(_k);
    _qdot.setZero(_k);
    _zero_vec_joint.setZero(_k);

    _A.setZero(_k,_k);
    _g.setZero(_k);
	_b.setZero(_k);
	_bg.setZero(_k);

	_mj_model = nullptr;
	_mj_data = nullptr;
}

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

	_k = m->nv - _qvel_offset;

	_A.setZero(_k, _k);
	_g.setZero(_k);
	_b.setZero(_k);
	_bg.setZero(_k);
	_q.setZero(_k);
	_qdot.setZero(_k);
	_zero_vec_joint.setZero(_k);

	_bool_model_update = true;
}

void CModel::update_kinematics(VectorXd & q, VectorXd & qdot)
{
	_q = q;
	_qdot = qdot;
	_bool_kinematics_update = true;
}

void CModel::update_dynamics()
{
    if (_bool_kinematics_update == true)
    {
		int nv = _mj_model->nv;
		mjtNum M[nv*nv];
        mj_fullM(_mj_model, M, _mj_data->qM);
        for (int i = 0; i < _k; i++)
            for (int j = 0; j < _k; j++)
                _A(i, j) = M[ (i+ _qvel_offset) * nv + (j+ _qvel_offset)];

        for (int i = 0; i < _k; i++)
            _bg(i) = _mj_data->qfrc_bias[i+ _qvel_offset];
    }
    _bool_dynamics_update = true;
}
