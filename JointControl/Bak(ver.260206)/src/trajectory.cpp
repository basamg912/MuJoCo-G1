#include "trajectory.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>

CTrajectory::CTrajectory()
{	
	Initialize();
}

CTrajectory::~CTrajectory()
{
}

void CTrajectory::set_size(int dof)
{
	_vector_size = dof;
	_init_pos.setZero(_vector_size);
	_init_vel.setZero(_vector_size);
	_goal_pos.setZero(_vector_size);
	_goal_vel.setZero(_vector_size);
	_init_rot.setZero(3,3);
	_goal_rot.setZero(3,3);
}

void CTrajectory::Initialize()
{
	_time_start = 0.0;
	_time = 0.0;
	_time_end = 0.0;
	_vector_size = 1; //default = 1
	_init_pos.setZero(_vector_size);
	_init_vel.setZero(_vector_size);
	_goal_pos.setZero(_vector_size);
	_goal_vel.setZero(_vector_size);
	_init_rot.setZero(3,3);
	_goal_rot.setZero(3,3);
	_bool_trajectory_complete = false;
}

void CTrajectory::reset_initial(double time0, VectorXd init_pos, VectorXd init_vel)
{
	check_vector_size(init_pos);
	check_vector_size(init_vel);

	_time_start = time0;
	_init_pos = init_pos;
	_init_vel = init_vel;

	_bool_trajectory_complete = false;
}

void CTrajectory::reset_initial_rotation(double time_0, const Eigen::Matrix3d &rotation_0)
{
	_time_start = time_0;
	_init_rot = rotation_0;

	// _rpy_start = CustomMath::GetBodyRotationAngle(rotation_0);

	_bool_trajectory_complete = false;
}

void CTrajectory::update_time(double time)
{
	_time = time;
}

void CTrajectory::update_goal(VectorXd goal_pos, VectorXd goal_vel, double goal_time)
{
	check_vector_size(goal_pos);
	check_vector_size(goal_vel);
	_goal_pos = goal_pos;
	_goal_vel = goal_vel;
	_time_end = goal_time;
}

void CTrajectory::update_goal_rotation(double goal_time,const Eigen::Matrix3d &rotation_f)
{
	_time_end = goal_time;	
	_goal_rot = rotation_f;
	// rpy_end = CustomMath::GetBodyRotationAngle(rotation_f);
	// cout<<"_goal_rot : "<<_goal_rot<<endl<<endl;
}

VectorXd CTrajectory::position_cubicSpline()
{
	VectorXd xd(_vector_size);

	if (_time <= _time_start)
	{
		xd = _init_pos;
	}
	else if (_time >= _time_end)
	{
		xd = _goal_pos;
	}
	else {
		xd = _init_pos + _init_vel * (_time - _time_start)
			+ (3.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start)) - 2.0 * _init_vel / (_time_end - _time_start) - _goal_vel / (_time_end - _time_start)) * (_time - _time_start) * (_time - _time_start)
			+ (-2.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start) * (_time_end - _time_start)) + (_init_vel + _goal_vel) / ((_time_end - _time_start) * (_time_end - _time_start))) * (_time - _time_start) * (_time - _time_start) * (_time - _time_start);
	}
	return xd;
}

double cubic(double time, double time_0, double time_f,double _init_pos, double _goal_pos, double _init_vel, double _goal_vel)
{
	double xd;

	if (time <= time_0)
	{
		xd = _init_pos;
	}
	else if (time >= time_f)
	{
		xd = _goal_pos;
	}
	else {
		xd = _init_pos + _init_vel * (time - time_0)
			+ (3.0 * (_goal_pos - _init_pos) / ((time_f - time_0) * (time_f - time_0)) - 2.0 * _init_vel / (time_f - time_0) - _goal_vel / (time_f - time_0)) * (time - time_0) * (time - time_0)
			+ (-2.0 * (_goal_pos - _init_pos) / ((time_f - time_0) * (time_f - time_0) * (time_f - time_0)) + (_init_vel + _goal_vel) / ((time_f - time_0) * (time_f - time_0))) * (time - time_0) * (time - time_0) * (time - time_0);
	}
	return xd;
}

double cubicdot(double _time, double _time_start, double _time_end,double _init_pos, double _goal_pos, double _init_vel, double _goal_vel)
{
	double xdotd;

	if (_time <= _time_start)
	{
		xdotd = _init_vel;
	}
	else if (_time >= _time_end)
	{
		xdotd = _goal_vel;
	}
	else {
		xdotd = _init_vel + 2.0 * (3.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start)) - 2.0 * _init_vel / (_time_end - _time_start) - _goal_vel / (_time_end - _time_start)) * (_time - _time_start)
			+ 3.0 * (-2.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start) * (_time_end - _time_start)) + (_init_vel + _goal_vel) / ((_time_end - _time_start) * (_time_end - _time_start))) * (_time - _time_start) * (_time - _time_start);
	}
	return xdotd;
}

VectorXd CTrajectory::velocity_cubicSpline()
{
	VectorXd xdotd(_vector_size);

	if (_time <= _time_start)
	{
		xdotd = _init_vel;
	}
	else if (_time >= _time_end)
	{
		xdotd = _goal_vel;
	}
	else {
		xdotd = _init_vel + 2.0 * (3.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start)) - 2.0 * _init_vel / (_time_end - _time_start) - _goal_vel / (_time_end - _time_start)) * (_time - _time_start)
			+ 3.0 * (-2.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start) * (_time_end - _time_start)) + (_init_vel + _goal_vel) / ((_time_end - _time_start) * (_time_end - _time_start))) * (_time - _time_start) * (_time - _time_start);
	}
	return xdotd;
}

void CTrajectory::check_vector_size(VectorXd X)
{	
	if (X.size() == _vector_size)
	{

	}
	else
	{
		cout << "Warning!!! -- Vector size in CTrajectory mismatch occured! --" << endl << endl;
	}
}

int CTrajectory::check_trajectory_complete() //1 = time when trajectory complete
{	
	int diff = 0;
	bool previous_bool = _bool_trajectory_complete;
	if (_time >= _time_end-0.0001 && _bool_trajectory_complete == false)
	{
		// cout<<"_time_end "<<_time_end<<endl;
		// cout<<"_time "<<_time<<endl;
		_bool_trajectory_complete = true;
		diff = 1;
	}

	return diff;
}

Matrix3d CTrajectory::rotationCubic()
{
	if (_time >= _time_end)
	{
		return _goal_rot;
	}
	else if (_time < _time_start)
	{
		return _init_rot;
	}
		// double tau = cubic(_time, _time_start, _time_end, 0, 1, 0, 0); // 기존
	double tau = (_time - _time_start)/(_time_end-_time_start); // 추가

		
	Eigen::Matrix3d rot_scaler_skew;
	rot_scaler_skew = (_init_rot.transpose() * _goal_rot).log();
	Eigen::Matrix3d result = _init_rot * (rot_scaler_skew * tau).exp();
	return result;
}
//,const Eigen::Vector3d &w_0, const Eigen::Vector3d &a_0,
Vector3d CTrajectory::rotationCubicDot()
{
    Eigen::Matrix3d r_skew;
    r_skew = (_init_rot.transpose() * _goal_rot).log();
	
    Eigen::Vector3d a, b, c, r;
    double tau = (_time - _time_start) / (_time_end - _time_start);
	double total_time = (_time_end - _time_start);
    r(0) = r_skew(2, 1);
    r(1) = r_skew(0, 2);
    r(2) = r_skew(1, 0);
	
	double r_Euclidean_norm = r(0)*r(0) + r(1)*r(1) + r(2)*r(2);
	r_Euclidean_norm = sqrt(r_Euclidean_norm);
	Matrix3d A, B;
	B.setZero(3,3);
	for(int i = 0; i<3; i++)
	{
		B(i,i) = 1;
	}
	A = B - (1-cos(r_Euclidean_norm))/(r_Euclidean_norm * r_Euclidean_norm) * r_skew + (r_Euclidean_norm - sin(r_Euclidean_norm)) / (r_Euclidean_norm * r_Euclidean_norm * r_Euclidean_norm) * r_skew * r_skew;
	
	// A.inverse()*w_1

    // c = w_0;
    // b = a_0 / 2;
    // a = r - b - c;
    Eigen::Vector3d rd;
    for (int i = 0; i < 3; i++)
    {
    	rd(i) = cubicdot(_time, _time_start, _time_end, 0, r(i), 0, 0);
    }
    // rd = _init_rot * (r / total_time); // 추가??
	rd = _init_rot * rd; // 기존
    // if (tau < 0) return w_0;
    if (tau > 1) return Eigen::Vector3d::Zero();
    return rd;//3 * a * pow(tau, 2) + 2 * b * tau + c;
}