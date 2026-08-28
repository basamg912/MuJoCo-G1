#include "controller.h"
#include "mapping.h"
#include <cmath>

// ============================================================================
// [튜닝] RL policy 가동 전(대기) default pose 유지용 high PD 스케일 (그룹별)
//   joint_kp/kd (= RL 학습 게인) 에 곱해지는 배율. 'R' 키 전 home pose 고정용.
//   headless MuJoCo 재현 결론:
//    - 다리(mj 0~13): x3 은 발목 처짐→전방 낙상. 정적 기립 임계 ~x18 이상
//      (x18~x24 안정, x30 부근 수치 발산) → x20 채택.
//    - 팔/허리(mj 14~30): 가벼워서 x20 이면 진동(흔들림). x2 면 조용+처짐 작음.
//   ※ 재현은 mujoco 3.6, 실제는 3.8 이므로 GUI 에서 최종 확인/미세조정.
// ============================================================================
static constexpr double PRERL_LEG_KP_SCALE = 20.0;  // 다리 (mj idx 0~13)
static constexpr double PRERL_LEG_KD_SCALE = 10.0;
static constexpr double PRERL_UP_KP_SCALE  = 2.0;   // 팔/허리 (mj idx 14~30)
static constexpr double PRERL_UP_KD_SCALE  = 1.0;
CController::CController()
{
	_k = 31; // for kapex
	_wl3_body_id = -1;
	_wl3_default_com.setZero();
	_wl3_com_offset.setZero();
	Initialize();
}

CController::~CController()
{
}

void CController::reset(){
	_t = 0.0;
	_pre_t = 0.0;
	_bool_init = true;
	_q_des = _q_home;
	_last_policy_time = -1.0;
	_rl_enabled = false; // 리셋 시 pre-RL 대기(high PD hold) 상태로 복귀
}

void CController::setModel(const mjModel* m, mjData* d)
{
	Model.set_mujoco_model(m, d);
	_obs.setMujocoModel(m, d);
	_attn_obs.setMujocoModel(m, d);
	_obs.reset();
	_attn_obs.reset();
	_last_action.setZero(31);
	cacheWl3ComInfo();
	applyWl3ComOffset();
}

void CController::setVelocityCommand(double vx, double vy, double wz)
{
	// ! 활성화된 정책의 obs 에만 명령 전달
	if (_attn_policy) {
		_attn_obs.setVelocityCommand(vx, vy, wz);
	} else {
		_obs.setVelocityCommand(vx, vy, wz);
	}
}

void CController::setComCommand(double dx, double dy, double dz)
{
	_wl3_com_offset << dx, dy, dz;
	if (_attn_policy) {
		_attn_obs.setComCommand(dx, dy, dz);
	} else {
		_obs.setComCommand(dx, dy, dz);
	}
	applyWl3ComOffset();
}

void CController::cacheWl3ComInfo()
{
	const mjModel* mj_model = Model.getMjModel();
	if (mj_model == nullptr)
	{
		_wl3_body_id = -1;
		_wl3_default_com.setZero();
		return;
	}

	_wl3_body_id = mj_name2id(mj_model, mjOBJ_BODY, "WL3");
	if (_wl3_body_id < 0)
	{
		std::cout << "[WARN] WL3 body not found, COM command disabled" << '\n';
		_wl3_default_com.setZero();
		return;
	}

	for (int i = 0; i < 3; i++)
	{
		_wl3_default_com(i) = mj_model->body_ipos[3 * _wl3_body_id + i];
	}
}

void CController::applyWl3ComOffset()
{
	mjModel* mj_model = const_cast<mjModel*>(Model.getMjModel());
	if (mj_model == nullptr || _wl3_body_id < 0)
	{
		return;
	}

	for (int i = 0; i < 3; i++)
	{
		mj_model->body_ipos[3 * _wl3_body_id + i] = _wl3_default_com(i) + _wl3_com_offset(i);
	}
}
// ! free joint 가상관절. pelvis 가 시뮬레이션 상에서 좌표가 고정되어있지않으니 freejoint 로 설정
// ! 나머지 관절들은 좌표계가 부모 링크 기준이라서 고정
void CController::set_default_pose(mjData* d){
	int offset = Model.get_qpos_offset();
	const mjModel* m = Model.getMjModel();

	// freejoint(7개) 초기값을 XML의 body pos/quat (m->qpos0) 에서 가져옴
	if (offset == 7){
		for (int i = 0; i < 7; i++){
			d->qpos[i] = m->qpos0[i];
		}
	}

	for (int i= offset; i< m->nq; i++){
		d->qpos[i] = 0.0;
	}

	// ! 하체
    d->qpos[offset + 2]  = -0.035;   // LLJ3
    d->qpos[offset + 3]  = 0.38;     // LLJ4 og : -0.38
    d->qpos[offset + 4]  = -0.33;    // LLJ5
    d->qpos[offset + 9]  = 0.035;    // RLJ3
    d->qpos[offset + 10] = -0.38;    // RLJ4 og : 0.38
    d->qpos[offset + 11] = 0.33;     // RLJ5

	// ! 팔
    d->qpos[offset + 17] = 0.2;      // LAJ1
    d->qpos[offset + 18] = 0.2;      // LAJ2
    d->qpos[offset + 19] = 0.18;     // LAJ3
    d->qpos[offset + 20] = -0.35;     // LAJ4
    d->qpos[offset + 24] = -0.2;     // RAJ1
    d->qpos[offset + 25] = -0.2;     // RAJ2
    d->qpos[offset + 26] = -0.18;    // RAJ3
    d->qpos[offset + 27] = 0.35;      // RAJ4

	// ! 관절 속도, ctrl 도 초기화
	for (int i=0; i< Model.getMjModel()->nv; i++) d->qvel[i] = 0.0;
	for (int i=0; i<Model.getMjModel()->nu; i++){
		if(i < 31) d->ctrl[i] = 0.0; // ! actuator 갯수만큼 for 문
	}
}

// ! main.cc 에서 읽은 MjData d 에서 q, qdot 을 읽어옴 (시뮬레이터에서)
void CController::read(double t, double* q, double* qdot)
{
	_t = t;
	if (_bool_init == true)
	{
		_init_t = _t;
		_bool_init = false;
	}

	_dt = t - _pre_t;
	// cout<<"_dt : "<<_dt<<endl;
	_pre_t = t;

	for (int i = 0; i < _k; i++)
	{
		_q(i) = q[i+ Model.get_qpos_offset()]; // ! free joint 7개 xyz 쿼터니언4개
		_qdot(i) = qdot[i+ Model.get_qvel_offset()]; // ! free joint 6개 xyz rpy
		_pre_q(i) = _q(i);
		_pre_qdot(i) = _qdot(i);
	}
}

void CController::write(double* ctrl)
{
	for (int i = 0; i < _k; i++)
	{
		int mj_idx = isaac_joint_to_mujoco[i];
		// RL 가동 전: 다리(0~13)=기립용 high PD, 팔/허리(14~30)=낮게(x20이면 진동).
		// RL 가동 후: 기존(슬라이더) 스케일.
		double kp_scale, kd_scale;
		if (_rl_enabled)      { kp_scale = _kp_scale;          kd_scale = _kd_scale; }
		else if (mj_idx < 14) { kp_scale = PRERL_LEG_KP_SCALE; kd_scale = PRERL_LEG_KD_SCALE; }
		else                  { kp_scale = PRERL_UP_KP_SCALE;  kd_scale = PRERL_UP_KD_SCALE; }
		// ! qdot_des = 0 이면, - kd * _qdot
		double torque = kp_scale * joint_kp[mj_idx] * (_q_des(mj_idx) - _q(mj_idx)) - kd_scale * joint_kd[mj_idx] * _qdot(mj_idx);

		constexpr bool KAPEX_ACTUATOR = false;
		const double Va = 0.01;
		if (KAPEX_ACTUATOR) {
		    if (mj_idx >= std::size(JOINT_ACTUATOR_MAP)) continue;
			const ActuatorParam* p = JOINT_ACTUATOR_MAP[mj_idx];
			if (p == nullptr){
    			torque = std::max( -joint_effort_limit[mj_idx], std::min(joint_effort_limit[mj_idx], torque));
          		ctrl[mj_idx] = torque;
                continue;
			}
			const ActuatorParam& param = *p;
			bool same_dir = (_qdot[mj_idx] * torque) > 0;
			double base_max = param.Y1;
			double max_eff = base_max;
			if (std::abs(_qdot[mj_idx]) >= param.X1){
			    double slope = -base_max / (param.X2 - param.X1);
				max_eff = std::max(0.0, slope*(std::abs(_qdot[mj_idx]) - param.X1) + base_max);
			}
			torque = std::clamp(torque, -max_eff, max_eff);
			torque -= param.SF * std::tanh(_qdot[mj_idx] / Va);
			torque -= param.DF * _qdot[mj_idx];
			ctrl[mj_idx] = torque;
		}
		else {
    		torque = std::max( -joint_effort_limit[mj_idx], std::min(joint_effort_limit[mj_idx], torque));
    		ctrl[mj_idx] = torque;
		}
	}
}
double compute_velocity_dependent_limit(double qdot, double torque, double X1, double X2, double Y1){
    bool same_dir = (qdot * torque) > 0;
    double base_max = Y1;
    if (std::abs(qdot) < X1)    return base_max;

    double slope = -base_max / (X2- X1);
    double limit = slope * (std::abs(qdot) - X1) + base_max;
    return std::max(0.0, limit);
}
void CController::control_mujoco()
{
    ModelUpdate(); // ! 동역학 계산

	if (_policy == nullptr && _attn_policy == nullptr){
		_q_des =  _q_home; // ! mujoco idx 순서
		_qdot_des.setZero();
		return;
	}

	// ! 'R' 키로 RL 활성화 전에는 전 관절 default pose 유지 (write() 의 high PD 가 고정)
	if (!_rl_enabled){
		_q_des = _q_home;
		_qdot_des.setZero();
		return;
	}

	if (_t - _last_policy_time >= 0.02 || _last_policy_time < 0){
		_last_policy_time = _t;
		static bool once = true;
		Eigen::VectorXd action;
		if (_attn_policy) {
			// ! attention 분기
			Eigen::MatrixXd joint_obs  = _attn_obs.computeJointObs(_q, _qdot, _q_home, _last_action);
			Eigen::MatrixXd feet_obs   = _attn_obs.computeFeetObs();
			Eigen::VectorXd global_obs = _attn_obs.computeGlobalObs();
			// if (once){
			//     cout << "[INFO] Global Obs (15 dim) : " << global_obs << ' ';
			// 				cout <<'\n';
			// }
			action = _attn_policy->inference(
				_attn_obs.jointDesc(),
				joint_obs,
				_attn_obs.feetDesc(),
				feet_obs,
				global_obs
			);
			// if (once){
			//     cout << "[INFO] Action (31 dim) : " << action << ' ';
			// 				once = false;
			// 				cout <<'\n';
			// }
		} else {
			// ! legacy MLP 분기
			Eigen::VectorXd stacked_obs = _obs.update(_q, _qdot, _q_home, _last_action);
			action = _policy->inference(stacked_obs); // policy 는 isaaclab 관절 순서대로
		}

		Eigen::VectorXd action_mj(_k);
		action_mj.setZero();
		if (action.size() == 14) {
			// ! leg-only 정책: 14 출력(isaac leg order) → mujoco 다리(0~13)
			for (int i=0; i<14; i++) action_mj(isaac_leg_to_mujoco[i]) = action(i);
		} else {
			// ! 31-out 정책: 전체 매핑 후 비다리(14~30) 마스킹
			for (int i=0; i< _k && i < action.size(); i++) action_mj(isaac_joint_to_mujoco[i]) = action(i);
			for (int mj = 14; mj < _k; mj++) action_mj(mj) = 0.0;
		}
		// 비다리(허리+양팔)는 action_mj=0 → _q_des=_q_home (RL 게인으로 default pose 유지)
		_last_action = action;
		_q_des = _q_home + 0.25 * action_mj;
	}
	_qdot_des.setZero();
	// ! position 제어 시 pd 제어는 무조코가 진행, -> 현재는 수동 토크 계산
	JointControl();
}

void CController::ModelUpdate()
{
    Model.update_kinematics(_q, _qdot);
	Model.update_dynamics();
    Model.calculate_EE_Jacobians();
	Model.calculate_EE_positions_orientations();
	Model.calculate_EE_velocity();

	_J_hands = Model._J_hand;

	_x_hand.head(3) = Model._x_hand;
	_x_hand.tail(3) = CustomMath::GetBodyRotationAngle(Model._R_hand);
	// cout << _x_hand.transpose() << endl;
	// cout << "R hand " << Model._R_hand.transpose() << endl;
	_xdot_hand = Model._xdot_hand;
}


void CController::JointControl()
{
	// _torque.setZero();
	// _A_diagonal = Model._A;

	// // ! armature 질량행렬=관성
	// // ! 대각원소에 1을 더해서 정규화, 대각값이 너무 작은 관절이 있으면 토크 불안정하기 때문
	// for(int i = 0; i < _k; i++){
	// 	_A_diagonal(i,i) += 1.0;
	// }
	// // ! Model._bg : 중력보상


	// _torque = _A_diagonal*(_kpj*(_q_des - _q) + _kdj*(_qdot_des - _qdot)) + Model._bg;

	// _torque = _kp_diag.cwiseProduct(_q_des - _q) + _kd_diag.cwiseProduct(_qdot_des - _qdot) + Model._bg;

	// position actuator 사용: PD 계산은 MuJoCo가 처리
	// _q_des는 control_mujoco()에서 trajectory로 업데이트됨
	// write()에서 _q_des를 d->ctrl로 전달
}


void CController::Initialize()
{
    _control_mode = 1; //1: joint space, 2: task space(CLIK)

	_bool_init = true;
	_t = 0.0;
	_init_t = 0.0;
	_pre_t = 0.0;
	_dt = 0.0;

	_kpj = 300.0;
	_kdj = 40.0;

	// _kpj_diagonal.setZero(_k, _k);
	// //							0 		1	2		3	   4	5 	6
	// _kpj_diagonal.diagonal() << 400., 2500., 1500., 1700., 700., 500., 520.;
	// _kdj_diagonal.setZero(_k, _k);
	// _kdj_diagonal.diagonal() << 20., 250., 170., 320., 70., 50., 15.;
	_x_kp = 1;//작게 0.1
	// _x_kp = 20.0;

    _q.setZero(_k);
	_qdot.setZero(_k);
	_torque.setZero(_k);

	_J_hands.setZero(6,_k);
	_J_bar_hands.setZero(_k,6);

	_x_hand.setZero(6);
	_xdot_hand.setZero(6);

	//////////////////원본///////////////////
	// _cnt_plan = 0;
	_bool_plan.setZero(30);
	// _time_plan.resize(30);
	// _time_plan.setConstant(5.0);
	//////////////////원본///////////////////

	_q_home.setZero(_k);
	// !
	// LL (인덱스 0~6)
	_q_home(2) = -0.035;   // LLJ3
	_q_home(3) = 0.38;     // LLJ4
	_q_home(4) = -0.33;    // LLJ5

	// RL (인덱스 7~13)
	_q_home(9)  = 0.035;   // RLJ3
	_q_home(10) = -0.38;   // RLJ4
	_q_home(11) = 0.33;    // RLJ5

	// WL (인덱스 14~16)
	// 전부 0

	// LA (인덱스 17~23)
	_q_home(17) = 0.2;     // LAJ1
	_q_home(18) = 0.2;     // LAJ2
	_q_home(19) = 0.18;    // LAJ3
	_q_home(20) = -0.35;    // LAJ4

	// RA (인덱스 24~30)
	_q_home(24) = -0.2;    // RAJ1
	_q_home(25) = -0.2;    // RAJ2
	_q_home(26) = -0.18;   // RAJ3
	_q_home(27) = 0.35;     // RAJ4

	_start_time = 0.0;
	_end_time = 0.0;
	_motion_time = 0.0;

	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_q_des.setZero(_k);
	// ! 발산 방지
	_q_des = _q_home;

	_qdot_des.setZero(_k);
	_q_goal.setZero(_k);
	_qdot_goal.setZero(_k);

	_x_des_hand.setZero(6);
	_xdot_des_hand.setZero(6);
	_x_goal_hand.setZero(6);
	_xdot_goal_hand.setZero(6);

	_pos_goal_hand.setZero(); // 3x1
	_rpy_goal_hand.setZero(); // 3x1
	JointTrajectory.set_size(_k);
	_A_diagonal.setZero(_k,_k);

	_x_err_hand.setZero(6);
	_R_des_hand.setZero();

	_I.setIdentity(_k,_k);

	_pre_q.setZero(_k);
	_pre_qdot.setZero(_k);

	_q_order.setZero(_k);
	_qdot_order.setZero(_k);
	_cnt_plan = 0;
	_bool_plan(_cnt_plan) = 1;
}
