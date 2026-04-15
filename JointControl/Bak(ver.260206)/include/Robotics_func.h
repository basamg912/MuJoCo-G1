/////////////////////////////////////////////////////////////////////////////
//
//	LinAlgebra.h : Common constant used in Math lib.
//
/////////////////////////////////////////////////////////////////////////////
#ifndef _ROBOTICS_FUNC_H_INCLUDED
#define _ROBOTICS_FUNC_H_INCLUDED

#include "CommUtil.h"


// ============================================================================================
//                    FUNCTION DEFINITION
// ============================================================================================

//===================== LinAlgebra functions =================================


template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> Skew(const Eigen::MatrixBase<Derived>& vec)
{
	EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

	Eigen::Matrix<typename Derived::Scalar, 3, 3> ret_vec;
	ret_vec <<	 0,		-vec(2), vec(1),
				 vec(2), 0,		-vec(0),
				-vec(1), vec(0), 0;
	return ret_vec;
}

/////	chatGPT ver.
template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, Derived::SizeAtCompileTime, 1> normalize(const Eigen::MatrixBase<Derived>& vec)
{
	typename Derived::Scalar norm = vec.matrix().norm();	//	Calculate the norm using the matrix() function
	return (vec / norm);									//	Return the normalized vector directly
}


//===================== filtering functions ===================================
template <typename T = sysReal>
inline Eigen::VectorXd _lowpass(const Eigen::VectorXd& previous, const Eigen::VectorXd& measure, const T& alpha)
{
	return (previous + (measure - previous) * alpha);
}

///	chatGPT ver.
template<typename Derived1, typename Derived2, typename T>
inline Eigen::Matrix<typename Derived1::Scalar, Derived1::RowsAtCompileTime, Derived1::ColsAtCompileTime> 
lowpassFilter(const Eigen::MatrixBase<Derived1>& previous, const Eigen::MatrixBase<Derived2>& measure, const T& alpha)
{
	EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Derived1, Derived2);
	return (1 - alpha) * previous + alpha * measure;
}


// /// Dr.Lee ver.

static double LPF(double dT, double Wc, double X, double preY) //sampling time, cutoff freq, input, previous output
{
    double tau = 1.0 / Wc;
    double y = tau / (tau + dT) * preY + dT / (tau + dT) * X;
    return y;
}



//=====================trajectory planning functions========================
template<typename T = sysReal>
inline Eigen::VectorXd _positionA2B(const T& time, const T& period, const Eigen::VectorXd& posA, const Eigen::VectorXd& posB)
{
	Eigen::VectorXd pos;

	if (time <= period)
		pos = posA + (posB - posA) * (1. - cos(M_PI * time / period)) * 0.5;
	else
		pos = posB;

	return pos;
}

template<typename T = sysReal>
inline Eigen::VectorXd _velocityA2B(const T& time, const T& period, const Eigen::VectorXd& posA, const Eigen::VectorXd& posB)
{
	Eigen::VectorXd vel;

	if (time <= period)
		vel = (posB - posA) * sin(M_PI * time / period) * (M_PI / period) * 0.5;

	return vel;
}


//===================== robotics functions ===================================
inline Eigen::Vector3d _Quat2Eul(const Eigen::Vector4d& quat)
{
	Eigen::Vector4d quatNorm(quat);
	quatNorm = normalize(quat);

	Eigen::Vector3d ret;
	ret(0) = atan2(2.0 * (quatNorm(2) * quatNorm(3) + quatNorm(0) * quatNorm(1)), quatNorm(0) * quatNorm(0) - quatNorm(1) * quatNorm(1) - quatNorm(2) * quatNorm(2) + quatNorm(3) * quatNorm(3));	// roll
	ret(1) = asin(-2.0 * (quatNorm(1) * quatNorm(3) - quatNorm(0) * quatNorm(2)));																						// pitch
	ret(2) = atan2(2.0 * (quatNorm(1) * quatNorm(2) + quatNorm(0) * quatNorm(3)), quatNorm(0) * quatNorm(0) + quatNorm(1) * quatNorm(1) - quatNorm(2) * quatNorm(2) - quatNorm(3) * quatNorm(3));	// yaw
	
	return ret;
}

inline Eigen::Vector4d _Eul2Quat(const Eigen::Vector3d& eul)
{
	sysReal cr2, cp2, cy2;	// cos_(roll/2), cos_(pitch/2), cos_(yaw/2)
	sysReal sr2, sp2, sy2;	// sin_(roll/2), sin_(pitch/2), sin_(yaw/2)

	cr2 = cos(eul(0) / 2.0);
	cp2 = cos(eul(1) / 2.0);
	cy2 = cos(eul(2) / 2.0);
	sr2 = sin(eul(0) / 2.0);
	sp2 = sin(eul(1) / 2.0);
	sy2 = sin(eul(2) / 2.0);

	Eigen::Vector4d ret;
	ret(0) = cy2 * cp2 * cr2 + sy2 * sp2 * sr2;
	ret(1) = cy2 * cp2 * sr2 - sy2 * sp2 * cr2;
	ret(2) = cy2 * sp2 * cr2 + sy2 * cp2 * sr2;
	ret(3) = sy2 * cp2 * cr2 - cy2 * sp2 * sr2;

	return ret;
}




inline Eigen::Vector3d _Rot2Eul(const Eigen::Matrix3d& R)
{
	if (R.rows() != 3 || R.cols() != 3) _ErrorMsg("_Rot2Eul() : size error");

	Eigen::Vector3d eul;

	eul(1) = asin(-R(2, 0));

	if (R(2, 0) == 1.0) {						//	th(Pitch, Y) = pi/2 rad
		eul(0) = 0.0;							//	psi(Yaw, X) = 0 rad
		eul(2) = atan2(R(0, 1), R(1, 1));		//	phi(Roll, Z)
	}
	else if (R(2, 0) == -1.0) {					//	th(Pitch = -pi/2 rad
		eul(0) = 0.0;							//	psi(Yaw, X) = 0 rad
		eul(2) = -atan2(R(0, 1), R(1, 1));		//	phi(Roll, Z)
	}
	else {
		eul(0) = atan2(R(2, 1) / cos(eul(1)), R(2, 2) / cos(eul(1)));
		eul(2) = atan2(R(1, 0) / cos(eul(1)), R(0, 0) / cos(eul(1)));
	}

	return eul;
}


inline Eigen::Vector3d Euldot2Omega(const Eigen::Vector3d& euler_angle, const Eigen::Vector3d& euler_angle_dot){
	Eigen::Matrix3d B_mat;
	Eigen::Vector3d omega;

	B_mat << 0, -sin(euler_angle(0)), cos(euler_angle(0))*sin(euler_angle(1)),
				0,  cos(euler_angle(0)), sin(euler_angle(0))*cos(euler_angle(1)),
				1,             0,               -sin(euler_angle(1));

	omega = B_mat*euler_angle_dot;

	return omega;
}

inline Eigen::Vector3d Omega2Euldot(const Eigen::Vector3d& euler_angle, const Eigen::Vector3d& omega) {
    // Eigen::Matrix3d B_mat;
    // B_mat << 0, -sin(euler_angle(0)), cos(euler_angle(0))*sin(euler_angle(1)),
    //          0,  cos(euler_angle(0)), sin(euler_angle(0))*cos(euler_angle(1)),
    //          1,             0,               -sin(euler_angle(1));
    
    // // B_mat의 역행렬을 계산하여 euler_angle_dot 구하기
    // Eigen::Vector3d euler_angle_dot = B_mat.inverse() * omega;
    // return euler_angle_dot;
	    Eigen::Matrix3d B_mat;
    B_mat << 0, -sin(euler_angle(0)), cos(euler_angle(0))*sin(euler_angle(1)),
             0,  cos(euler_angle(0)), sin(euler_angle(0))*cos(euler_angle(1)),
             1,             0,               -sin(euler_angle(1));
    
    // Damped least squares: (B^T * B + λI)^(-1) * B^T * ω
    double lambda = 0.01;  // damping factor
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d damped = B_mat.transpose() * B_mat + lambda * I;
    
    Eigen::Vector3d euler_angle_dot = damped.inverse() * B_mat.transpose() * omega;
    return euler_angle_dot;
}
inline Eigen::Vector3d _Rot2EulZXY(const Eigen::Matrix3d& R)
{
	// Euler Angle Formulas
	// @article{eberly2008euler,
	// 	title = { Euler angle formulas },
	// 	author = { Eberly, David },
	// 	journal = { Geometric Tools, LLC, Technical Report },
	// 	pages = { 1--18 },
	// 	year = { 2008 }
	// }
	
	if (R.rows() != 3 || R.cols() != 3) _ErrorMsg("_Eul2RotZXY : size error");

	Eigen::Vector3d eul;

	if (R(2, 1) < 1.0)
	{
		if (R(2, 1) > -1.0)
		{
			eul(1) = asin(R(2, 1));
			eul(0) = atan2(-R(0, 1), R(1, 1));
			eul(2) = atan2(-R(2, 0), R(2, 2));
		}
		else
		{
			eul(1) = -M_PI / 2.0;
			eul(0) = -atan2(R(0, 2), R(0, 0));
			eul(2) = 0.0;
		}
	}
	else
	{
		eul(1) = M_PI / 2.0;
		eul(0) = atan2(R(0, 2), R(0, 0));
		eul(2) = 0.0;
	}

	return eul;
}

inline Eigen::Matrix3d _EulZXY2Rot(const Eigen::Vector3d& eul)
{
	if (eul.size() != 3) _ErrorMsg("_Eul2Rot : size error");

	// euler = (z, x, y)
	sysReal cos_z = cos(eul(0));
	sysReal sin_z = sin(eul(0));
	sysReal cos_x = cos(eul(1));
	sysReal sin_x = sin(eul(1));
	sysReal cos_y = cos(eul(2));
	sysReal sin_y = sin(eul(2));

	Eigen::Matrix3d mat;

	mat << 	cos_y * cos_z - sin_x * sin_y * sin_z, -cos_x * sin_z,  cos_z * sin_y + cos_y * sin_x * sin_z,
			cos_z * sin_x * sin_y + cos_y * sin_z,  cos_x * cos_z, -cos_y * cos_z * sin_x + sin_y * sin_z,
								   -cos_x * sin_y,          sin_x,                          cos_x * cos_y ;

	return mat;
}

inline Eigen::Matrix3d _Eul2Rot(const Eigen::Vector3d& eul)
{
	if (eul.size() != 3) _ErrorMsg("_Eul2Rot() : size error");

	//	euler = (z, y, x)
	sysReal cos_x = cos(eul(0));
	sysReal sin_x = sin(eul(0));
	sysReal cos_y = cos(eul(1));
	sysReal sin_y = sin(eul(1));
	sysReal cos_z = cos(eul(2));
	sysReal sin_z = sin(eul(2));

	Eigen::Matrix3d mat;

	mat << cos_x * cos_y, cos_x * sin_y * sin_z - sin_x * cos_z, cos_x * sin_y * cos_z + sin_x * sin_z,
		   sin_x * cos_y, sin_x * sin_y * sin_z + cos_x * cos_z, sin_x * sin_y * cos_z - cos_x * sin_z,
				  -sin_y,                         cos_y * sin_z,                         cos_y * cos_z ;

	return mat;
}

template<typename T = sysReal>
inline Eigen::Vector4d _Rot2Quat(const Eigen::Matrix3d& R)
{
	if (R.rows() != 3 || R.cols() != 3) _ErrorMsg("_Rot2Quat() : size error");

	Eigen::Vector4d quat;

	quat(0) = (R(0, 0) + R(1, 1) + R(2, 2) + 1.0) / 4.0;
	quat(1) = (R(0, 0) - R(1, 1) - R(2, 2) + 1.0) / 4.0;
	quat(2) = (-R(0, 0) + R(1, 1) - R(2, 2) + 1.0) / 4.0;
	quat(3) = (-R(0, 0) - R(1, 1) + R(2, 2) + 1.0) / 4.0;

	for (int i = 0; i < 4; i++) {
		if (quat(i) < 0.0)	quat(i) = 0.0;
		quat(i) = sqrt(quat(i));
	}

	if (quat(0) < ZCE) {
		if (quat(1) < ZCE) {
			quat(0) = 0.0;
			quat(1) = 0.0;
			quat(2) *= +1.0;
			quat(3) *= SGN(R(2, 1) + R(1, 2));
		}
		else {
			quat(0) = 0.0;
			quat(1) *= +1.0;
			quat(2) *= SGN(R(1, 0) + R(0, 1));
			quat(3) *= SGN(R(0, 2) + R(2, 0));
		}
	}
	else {
		quat(0) *= +1.0;
		quat(1) *= SGN(R(2, 1) - R(1, 2));
		quat(2) *= SGN(R(0, 2) - R(2, 0));
		quat(3) *= SGN(R(1, 0) - R(0, 1));
	}

	T r = sqrt(pow(quat(0), 2) + pow(quat(1), 2) + pow(quat(2), 2) + pow(quat(3), 2));

	if (r<ZCE && r>-ZCE)
		_ErrorMsg("_Rot2Quat : zero division error");
	else {
		quat(0) /= r;
		quat(1) /= r;
		quat(2) /= r;
		quat(3) /= r;

	}
	return quat;
}

inline Eigen::Matrix3d _Quat2Rot(const Eigen::Vector4d& quat)
{
	if (quat.size() != 4) _ErrorMsg("_Quat2Rot() : bad input size");
	Eigen::Matrix3d mat;

	mat(0, 0) = quat(0) * quat(0) + quat(1) * quat(1) - quat(2) * quat(2) - quat(3) * quat(3);
	mat(0, 1) = 2 * (quat(1) * quat(2) - quat(0) * quat(3));
	mat(0, 2) = 2 * (quat(1) * quat(3) + quat(0) * quat(2));

	mat(1, 0) = 2 * (quat(1) * quat(2) + quat(0) * quat(3));
	mat(1, 1) = quat(0) * quat(0) - quat(1) * quat(1) + quat(2) * quat(2) - quat(3) * quat(3);
	mat(1, 2) = 2 * (quat(2) * quat(3) - quat(0) * quat(1));

	mat(2, 0) = 2 * (quat(1) * quat(3) - quat(0) * quat(2));
	mat(2, 1) = 2 * (quat(2) * quat(3) + quat(0) * quat(1));
	mat(2, 2) = quat(0) * quat(0) - quat(1) * quat(1) - quat(2) * quat(2) + quat(3) * quat(3);

	return mat;
}

inline Eigen::Vector4d _quatMultiplicate(const Eigen::Vector4d& quat1, const Eigen::Vector4d& quat2)
{
	if (quat1.size() != 4 || quat2.size() != 4) _ErrorMsg("_quatMultiplicate : bad input size");
	Eigen::Vector4d quat;

	quat(0) = quat1(0) * quat2(0) - quat1(1) * quat2(1) - quat1(2) * quat2(2) - quat1(3) * quat2(3);
	quat(1) = quat1(0) * quat2(1) + quat1(1) * quat2(0) + quat1(2) * quat2(3) - quat1(3) * quat2(2);
	quat(2) = quat1(0) * quat2(2) - quat1(1) * quat2(3) + quat1(2) * quat2(0) + quat1(3) * quat2(1);
	quat(3) = quat1(0) * quat2(3) + quat1(1) * quat2(2) - quat1(2) * quat2(1) + quat1(3) * quat2(0);

	return quat;
}

inline Eigen::Vector4d _quatDivide(const Eigen::Vector4d& quat1, const Eigen::Vector4d& quat2)
{
	if (quat1.size() != 4 || quat2.size() != 4) _ErrorMsg("_quatDivide : bad input size");
	Eigen::Vector4d quat;

	quat(0) = quat1(0) * quat2(0) + quat1(1) * quat2(1) + quat1(2) * quat2(2) + quat1(3) * quat2(3);
	quat(1) = -quat1(0) * quat2(1) + quat1(1) * quat2(0) - quat1(2) * quat2(3) + quat1(3) * quat2(2);
	quat(2) = -quat1(0) * quat2(2) + quat1(1) * quat2(3) + quat1(2) * quat2(0) - quat1(3) * quat2(1);
	quat(3) = -quat1(0) * quat2(3) - quat1(1) * quat2(2) + quat1(2) * quat2(1) + quat1(3) * quat2(0);

	return quat;
}



/// @brief q_cur → q_des 로 가는 오차 쿼터니언을 계산합니다.
/// @param q_des 목표 자세 (Eigen::Quaterniond)
/// @param q_cur 현재 자세 (Eigen::Quaterniond)
/// @return 단위(quat.w>=0) 오차 쿼터니언 q_err
inline Eigen::Quaterniond quatError(
    const Eigen::Quaterniond &q_des,
    const Eigen::Quaterniond &q_cur
) {
    // 1) 오차 = des * inv(cur)  (unit quat 이면 inv = conjugate)
    Eigen::Quaterniond q_err = q_des * q_cur.conjugate();

    // 2) 부호 정규화: 항상 w>=0 으로 만들어 최단 회전 보장
    if (q_err.w() < 0.0) {
        q_err.coeffs() *= -1.0;
    }

    // 3) 혹시 모를 수치 오차 제거를 위해 정규화
    return q_err.normalized();
}



// quat error to euler error
template<typename T = sysReal>
inline Eigen::Vector3d _quatError(const Eigen::Vector4d& quat1, const Eigen::Vector4d& quat2)
{
	// (1) multiplying scalar value sign: SIGN AMBIGUITY can be avoided!
	Eigen::Vector4d quatErr;
	quatErr = _quatDivide(quat1, quat2);

	// (2) using angular error transformation matrix: SINGULARITY at theta=90deg
	Eigen::Matrix3d quatErr0Mat;
	quatErr0Mat.setIdentity();
	quatErr0Mat *= quatErr(0);

	Eigen::Vector3d quatErrEps;
	quatErrEps = quatErr.segment(1, 3);

	Eigen::MatrixXd U = (quatErr0Mat + Skew(quatErrEps)) * 0.5;

	return (U.transpose() * quatErrEps);
}

inline Eigen::Vector4d _quatConjugate(const Eigen::Vector4d& quat1)
{
	Eigen::Vector4d quatTemp;
	quatTemp(0) = quat1(0);
	quatTemp(1) = -quat1(1);
	quatTemp(2) = -quat1(2);
	quatTemp(3) = -quat1(3);

	return quatTemp;
}

inline Eigen::Vector4d _quatInv(const Eigen::Vector4d& quat1)
{
	sysReal quatSQR(quat1(0) * quat1(0)
				 + quat1(1) * quat1(1)
				 + quat1(2) * quat1(2)
				 + quat1(3) * quat1(3));

	return (_quatConjugate(quat1) / quatSQR);
}

inline Eigen::Matrix3d _Rot_X(const sysReal& q)
{
	sysReal c, s;
	s = sin(q);
	c = cos(q);

	Eigen::Matrix3d rot;
	rot(0, 0) = 1.0;	rot(0, 1) = 0.0;	rot(0, 2) = 0.0;
	rot(1, 0) = 0.0;	rot(1, 1) = c;		rot(1, 2) = -s;
	rot(2, 0) = 0.0;	rot(2, 1) = s;		rot(2, 2) = c;

	return rot;
}

inline Eigen::Matrix3d _Rot_Y(const sysReal& q)
{
	sysReal c, s;

	c = cos(q);		s = sin(q);

	Eigen::Matrix3d rot;
	rot(0, 0) = c;		rot(0, 1) = 0.0;	rot(0, 2) = s;
	rot(1, 0) = 0.0;	rot(1, 1) = 1.0;	rot(1, 2) = 0.0;
	rot(2, 0) = -s;		rot(2, 1) = 0.0;	rot(2, 2) = c;

	return rot;
}

inline Eigen::Matrix3d _Rot_Z(const sysReal& q)
{
	sysReal c, s;
	c = cos(q);		s = sin(q);

	Eigen::Matrix3d rot;
	rot(0, 0) = c;		rot(0, 1) = -s;		rot(0, 2) = 0.0;
	rot(1, 0) = s;		rot(1, 1) = c;		rot(1, 2) = 0.0;
	rot(2, 0) = 0.0;	rot(2, 1) = 0.0;	rot(2, 2) = 1.0;

	return rot;
}

inline Eigen::Matrix3d _Rot_RPY(const sysReal& z, const sysReal& y, const sysReal& x)
{
	// successive rotation about moving coordinate frame.
	// roll-pitch-yaw

	sysReal c_z = cos(z);
	sysReal s_z = sin(z);
	sysReal c_y = cos(y);
	sysReal s_y = sin(y);
	sysReal c_x = cos(x);
	sysReal s_x = sin(x);

	Eigen::Matrix3d rot;
	rot(0, 0) = c_z * c_y;		rot(0, 1) = c_z * s_y * s_x - s_z * c_x;		rot(0, 2) = c_z * s_y * c_x + s_z * s_x;
	rot(0, 0) = s_z * c_y;		rot(0, 1) = s_z * s_y * s_x + c_z * c_x;		rot(0, 2) = s_z * s_y * c_x - c_z * s_x;
	rot(0, 0) = -s_y;			rot(0, 1) = c_y * s_x;							rot(0, 2) = c_y * c_x;

	return rot;
}


/* //////////////////////////////////////////////////////////////////////////////// */
/* 		orientation conversion														*/
/*   		: Yan-Bin Jia, "Quaternions and Rotations," technical reports, 2015		*/
/* //////////////////////////////////////////////////////////////////////////////// */
inline Eigen::Vector3d convert_quatDot2angVel(Eigen::Vector4d quat, Eigen::Vector4d quatDot)
{
	Eigen::Vector4d omegaQuat;
	omegaQuat = 2 * _quatMultiplicate(quatDot, _quatInv(quat));
	return { omegaQuat(1), omegaQuat(2), omegaQuat(3) };
}

inline Eigen::Vector3d	convert_quatDdot2angAcc(Eigen::Vector4d quat, Eigen::Vector4d quatDot, Eigen::Vector4d quatDdot)
{
	Eigen::Vector4d omegaDotQuat = 2 * (_quatMultiplicate(quatDdot, _quatInv(quat)) - _quatMultiplicate(_quatMultiplicate(quatDot, _quatInv(quat)), _quatMultiplicate(quatDot, _quatInv(quat))));
	return { omegaDotQuat(1), omegaDotQuat(2), omegaDotQuat(3) };
}

inline Eigen::Vector4d convert_angVel2quatDot(Eigen::Vector4d quat, Eigen::Vector3d omega)
{
	Eigen::Vector4d omegaQuat = {0, omega(0), omega(1), omega(2)};
	Eigen::Vector4d ret;
	ret = (0.5 * ret.setOnes() * omegaQuat.transpose())*quat;
	return ret;
}

inline Eigen::Vector4d convert_angAcc2quatDdot(Eigen::Vector4d quat, Eigen::Vector3d omega, Eigen::Vector3d omegaDot)
{
	Eigen::Vector4d omegaQuat = { 0, omega(0), omega(1), omega(2) };
	Eigen::Vector4d omegaDotQuat = { 0, omegaDot(0), omegaDot(1), omegaDot(2) };
	Eigen::Vector4d ret;
	ret = ret.setOnes() * (0.5 * (omegaDotQuat + ret.setOnes() * ((0.5 * omegaQuat.transpose()) * omegaQuat)).transpose() * quat);

	return ret;
}


	// Adaptive SVD-based damped pseudo-inverse function
inline	Eigen::MatrixXd computeAdaptiveDampedPseudoInverse(const Eigen::MatrixXd& jacobian, 
                                                                  double lambda_base, 
                                                                  double lambda_max, 
                                                                  double condition_threshold,
                                                                  bool verbose) {
    // SVD decomposition
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXd singular_values = svd.singularValues();
    
    // Adaptive damping based on singular values
    double sigma_min = singular_values.minCoeff();
    double sigma_max = singular_values.maxCoeff();
    double condition_number = sigma_max / (sigma_min + 1e-12);  // avoid division by zero
    
    // Adaptive lambda based on condition number
    double lambda = (condition_number > condition_threshold) ? lambda_max : lambda_base;


    if (verbose) {
        std::cout << "condition_number: " << condition_number << std::endl;
        std::cout << "lambda: " << lambda << std::endl;
    }
    
    // Damped pseudo-inverse using SVD
    Eigen::MatrixXd Sigma_inv = Eigen::MatrixXd::Zero(jacobian.cols(), jacobian.rows());
    for (int i = 0; i < singular_values.size(); ++i) {
        double sigma = singular_values(i);
        Sigma_inv(i, i) = sigma / (sigma * sigma + lambda * lambda);
    }
    
    return svd.matrixV() * Sigma_inv * svd.matrixU().transpose();
}


#endif	// ++++++++++ End of #ifndef ++++++++++