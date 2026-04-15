#pragma once
#ifndef CUSTOMMATH_H
#define CUSTOMMATH_H

#define DEG2RAD (0.01745329251994329576923690768489)
#define RAD2DEG 1/DEG2RAD
#define GRAVITY 9.80665
#define PI 3.1415926535897932384626433
#define CustomArraySize 50

#include "math.h"
#include <eigen3/Eigen/Dense>
using namespace Eigen;

// namespace Eigen
// {
// 		typedef Matrix<double, -1, -1, 0, CustomArraySize, CustomArraySize> MatrixCXd;
// 		typedef Matrix<double, -1, -1, 0, CustomArraySize, 1> VectorCXd;
//         typedef Matrix<int, -1, -1, 0, CustomArraySize, CustomArraySize> MatrixCXi;
// 		typedef Matrix<int, -1, -1, 0, CustomArraySize, 1> VectorCXi;
//         typedef Matrix<bool, -1, -1, 0, CustomArraySize, CustomArraySize> MatrixCXb;
// 		typedef Matrix<bool, -1, -1, 0, CustomArraySize, 1> VectorCXb;

//         typedef Matrix<double, -1, -1, 0, 900, 1> VectorCXde;
//         typedef Matrix<int, -1, -1, 0, 900, 1> VectorCXie;

//         typedef Matrix<double, -1, -1, 0, 50000, 1> VectorYYD;
//         typedef Matrix<double, -1, -1, 0, 50000, 50000> MatrixYY;
//         typedef Matrix<int, -1, -1, 0, 50000, 1> VectorYYI;
// }

namespace CustomMath
{
    // // Dr. Y H OH

    // static Eigen::Matrix3d _Quat2Rot(const Eigen::Vector4d& quat)
    // {
    // 	Eigen::Matrix3d mat;

    // 	mat(0, 0) = quat(0) * quat(0) + quat(1) * quat(1) - quat(2) * quat(2) - quat(3) * quat(3);
    // 	mat(0, 1) = 2 * (quat(1) * quat(2) - quat(0) * quat(3));
    // 	mat(0, 2) = 2 * (quat(1) * quat(3) + quat(0) * quat(2));

    // 	mat(1, 0) = 2 * (quat(1) * quat(2) + quat(0) * quat(3));
    // 	mat(1, 1) = quat(0) * quat(0) - quat(1) * quat(1) + quat(2) * quat(2) - quat(3) * quat(3);
    // 	mat(1, 2) = 2 * (quat(2) * quat(3) - quat(0) * quat(1));

    // 	mat(2, 0) = 2 * (quat(1) * quat(3) - quat(0) * quat(2));
    // 	mat(2, 1) = 2 * (quat(2) * quat(3) + quat(0) * quat(1));
    // 	mat(2, 2) = quat(0) * quat(0) - quat(1) * quat(1) - quat(2) * quat(2) + quat(3) * quat(3);

    // 	return mat;
    // }

    // static Eigen::Vector4d _Rot2Quat(const Eigen::Matrix3d& R)
    // {
    // 	Eigen::Vector4d quat;
    
    // 	quat(0) = (R(0, 0) + R(1, 1) + R(2, 2) + 1.0) / 4.0;
    // 	quat(1) = (R(0, 0) - R(1, 1) - R(2, 2) + 1.0) / 4.0;
    // 	quat(2) = (-R(0, 0) + R(1, 1) - R(2, 2) + 1.0) / 4.0;
    // 	quat(3) = (-R(0, 0) - R(1, 1) + R(2, 2) + 1.0) / 4.0;
    
    // 	for (int i = 0; i < 4; i++) {
    // 		if (quat(i) < 0.0)	quat(i) = 0.0;
    // 		quat(i) = sqrt(quat(i));
    // 	}
    
    // 	if (quat(0) < ZCE) {
    // 		if (quat(1) < ZCE) {
    // 			quat(0) = 0.0;
    // 			quat(1) = 0.0;
    // 			quat(2) *= +1.0;
    // 			quat(3) *= SGN(R(2, 1) + R(1, 2));
    // 		}
    // 		else {
    // 			quat(0) = 0.0;
    // 			quat(1) *= +1.0;
    // 			quat(2) *= SGN(R(1, 0) + R(0, 1));
    // 			quat(3) *= SGN(R(0, 2) + R(2, 0));
    // 		}
    // 	}
    // 	else {
    // 		quat(0) *= +1.0;
    // 		quat(1) *= SGN(R(2, 1) - R(1, 2));
    // 		quat(2) *= SGN(R(0, 2) - R(2, 0));
    // 		quat(3) *= SGN(R(1, 0) - R(0, 1));
    // 	}
    
    // 	T r = sqrt(pow(quat(0), 2) + pow(quat(1), 2) + pow(quat(2), 2) + pow(quat(3), 2));
    
    // 	if (r<ZCE && r>-ZCE)
    // 		_ErrorMsg("_Rot2Quat : zero division error");
    // 	else {
    // 		quat(0) /= r;
    // 		quat(1) /= r;
    // 		quat(2) /= r;
    // 		quat(3) /= r;
        
    // 	}
    // 	return quat;
    // }

    // inline Eigen::Vector3d _Rot2Eul(const Eigen::Matrix3d& R)
    // {
    // 	Eigen::Vector3d eul;

    // 	eul(1) = asin(-R(2, 0));

    // 	if (R(2, 0) == 1.0) {						//	th(Pitch, Y) = pi/2 rad
    // 		eul(0) = 0.0;							//	psi(Yaw, X) = 0 rad
    // 		eul(2) = atan2(R(0, 1), R(1, 1));		//	phi(Roll, Z)
    // 	}
    // 	else if (R(2, 0) == -1.0) {					//	th(Pitch = -pi/2 rad
    // 		eul(0) = 0.0;							//	psi(Yaw, X) = 0 rad
    // 		eul(2) = -atan2(R(0, 1), R(1, 1));		//	phi(Roll, Z)
    // 	}
    // 	else {
    // 		eul(0) = atan2(R(2, 1) / cos(eul(1)), R(2, 2) / cos(eul(1)));
    // 		eul(2) = atan2(R(1, 0) / cos(eul(1)), R(0, 0) / cos(eul(1)));
    // 	}

    // 	return eul;
    // }

    // static Eigen::Matrix3d _Rot_Z(const double& q)
    // {
    // 	double c, s;
    // 	c = cos(q);		s = sin(q);

    // 	Eigen::Matrix3d rot;
    // 	rot(0, 0) = c;		rot(0, 1) = -s;		rot(0, 2) = 0.0;
    // 	rot(1, 0) = s;		rot(1, 1) = c;		rot(1, 2) = 0.0;
    // 	rot(2, 0) = 0.0;	rot(2, 1) = 0.0;	rot(2, 2) = 1.0;

    // 	return rot;
    // }

    // pseudo inverse
    static MatrixXd pseudoInverseSVD(const MatrixXd& a, double epsilon = std::numeric_limits<double>::epsilon())
    {
        JacobiSVD<MatrixXd> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);

        MatrixXd ainv(a.cols(), a.rows());
        ainv.noalias() = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();

        return ainv;
    }

    static MatrixXd pseudoInverseQR(const MatrixXd& A)
    {
        CompleteOrthogonalDecomposition<MatrixXd> cod(A);
        return cod.pseudoInverse();
    }

    static MatrixXd OneSidedInverse(const MatrixXd& A) //no use
    {
        MatrixXd Ainv(A.cols(), A.rows());

        if (A.cols() > A.rows()) //left inverse
        {
            MatrixXd AAT(A.rows(), A.rows());
            AAT = A * A.transpose();
            MatrixXd AATinv(A.rows(), A.rows());
            AATinv = AAT.inverse();

            Ainv = A.transpose() * AATinv;
        }
        else //right inverse
        {
            MatrixXd ATA(A.cols(), A.cols());
            ATA = A.transpose() * A;
            MatrixXd ATAinv(A.cols(), A.cols());
            ATAinv = ATA.inverse();
            Ainv = ATAinv * A.transpose();
        }

        return Ainv;
    }

    static MatrixXd WeightedPseudoInverse(const MatrixXd& Q, const MatrixXd& W, const bool Is_W_fullrank)
    {
        MatrixXd invQ(Q.cols(), Q.rows());
        MatrixXd transposeQ(Q.cols(), Q.rows());
        MatrixXd Winv(W.cols(), W.rows());
        if (Is_W_fullrank == true)
        {
            Winv = W.inverse();
        }
        else
        {
            Winv = pseudoInverseQR(W);
            //Winv = pseudoInverseSVD(W, 0.0001);
        }
        transposeQ = Q.transpose();
        MatrixXd tmp1(W.cols(), Q.rows());
        tmp1.noalias() = Winv * transposeQ;
        MatrixXd tmp2(Q.rows(), W.rows());
        tmp2.noalias() = Q * Winv;
        MatrixXd tmp3(Q.rows(), Q.rows());
        tmp3.noalias() = tmp2 * transposeQ;
        MatrixXd tmp3inv(Q.rows(), Q.rows());
        tmp3inv = tmp3.inverse();
        invQ.noalias() = tmp1 * tmp3inv;
        //invQ = Winv*transposeQ*pseudoInverseQR(Q*Winv*transposeQ);
        //invQ = Winv*transposeQ*pseudoInverseSVD(Q*Winv*transposeQ);

        return invQ;
    }

    static MatrixXd DampedWeightedPseudoInverse(const MatrixXd& Q, const MatrixXd& W, const bool Is_W_fullrank, double sigma)
    {
        MatrixXd invQ(Q.cols(), Q.rows());
        MatrixXd transposeQ(Q.cols(), Q.rows());
        MatrixXd Winv(W.cols(), W.rows());
        if (Is_W_fullrank == true)
        {
            Winv = W.inverse();
        }
        else
        {
            Winv = pseudoInverseQR(W);
            //Winv = pseudoInverseSVD(W, 0.0001);
        }
        transposeQ = Q.transpose();
        // double sigma = 0.01;
        sigma = 0.5;
        MatrixXd Id(Q.rows(), Q.rows());
        Id.setIdentity();
        //invQ = Winv*transposeQ*pseudoInverseQR(Q*Winv*transposeQ + sigma*Id);
        MatrixXd tmp1(W.cols(), Q.rows());
        tmp1.noalias() = Winv * transposeQ;
        MatrixXd tmp2(Q.rows(), W.rows());
        tmp2.noalias() = Q * Winv;
        MatrixXd tmp3(Q.rows(), Q.rows());
        tmp3.noalias() = tmp2 * transposeQ;
        MatrixXd tmp4(Q.rows(), Q.rows());
        tmp4.noalias() = tmp3 + sigma * Id;
        MatrixXd tmp4inv(Q.rows(), Q.rows());
        //tmp4inv = pseudoInverseSVD(tmp4, 0.0001);
        tmp4inv = pseudoInverseQR(tmp4);

        invQ.noalias() = tmp1 * tmp4inv;

        return invQ;
    }

    // lowpass filter
    static double VelLowpassFilter(double rT, //delT
        double rWn, //cutoff freq (rad/sec)
        double rQ_pre, //previous position
        double rQ, //current position
        double rVel_pre //previous filtered velocity
    )
    {
        double rA;
        double rB;
        double rC;
        double rD;

        double rVel;

        rA = 2.0 * rWn;
        rB = (-2.0) * rWn;
        rC = 2.0 + rT * rWn;
        rD = rT * rWn - 2.0;

        rVel = ((-(rD)) * rVel_pre + (rA)*rQ + (rB)*rQ_pre) / (rC);

        return(rVel);
    }

    static double LowPassFilter(double dT, double Wc, double X, double preY) //sampling time, cutoff freq, input, previous output
    {
        double tau = 1.0 / Wc;
        double y = tau / (tau + dT) * preY + dT / (tau + dT) * X;
        return y;
    }

    static Eigen::Vector3d GetBodyRotationAngle(Eigen::Matrix3d RotMat)
    {
        double rollangle, pitchangle, yawangle;
        Eigen::Vector3d BodyAngle;
        BodyAngle.setZero();

        //  pitchangle = atan2(-RotMat(2,0),sqrt(RotMat(0,0)*RotMat(0,0)+RotMat(1,0)*RotMat(1,0)));
        //  yawangle = atan2(RotMat(1,0)/cos(pitchangle),RotMat(0,0)/cos(pitchangle));
        //  rollangle = atan2(RotMat(2,1)/cos(pitchangle), RotMat(2,2)/cos(pitchangle));

        double threshold = 0.001;
        pitchangle = -asin(RotMat(2, 0));
        if (RotMat(2, 0) > 1.0 - threshold && RotMat(2, 0) < 1.0 + threshold) //when RotMat(2,0) == 1
        {//Gimbal lock, pitch = -90deg
            rollangle = atan2(-RotMat(0, 1), -RotMat(0, 2));
            yawangle = 0.0;
        }
        else if (RotMat(2, 0) < -1.0 + threshold && RotMat(2, 0) > -1.0 - threshold) //when RotMat(2,0) == -1
        {//Gimbal lock, pitch = 90deg
            rollangle = atan2(RotMat(0, 1), RotMat(0, 2));
            yawangle = 0.0;
        }
        else //general solution
        {
            rollangle = atan2(RotMat(2, 1), RotMat(2, 2));
            yawangle = atan2(RotMat(1, 0), RotMat(0, 0));
        }

        BodyAngle(0) = rollangle;
        BodyAngle(1) = pitchangle;
        BodyAngle(2) = yawangle;

        return BodyAngle;
    }

    static double GetBodyPitchAngle(Eigen::Matrix3d RotMat)
    {
        double pitchangle;
        pitchangle = -asin(RotMat(2, 0));

        return pitchangle;
    }

    static double GetBodyRollAngle(Eigen::Matrix3d RotMat)
    {
        double rollangle, pitchangle;
        double threshold = 0.001;
        pitchangle = -asin(RotMat(2, 0));
        if (RotMat(2, 0) > 1.0 - threshold && RotMat(2, 0) < 1.0 + threshold) //when RotMat(2,0) == 1
        {//Gimbal lock, pitch = -90deg
            rollangle = atan2(-RotMat(0, 1), -RotMat(0, 2));
        }
        else if (RotMat(2, 0) < -1.0 + threshold && RotMat(2, 0) > -1.0 - threshold) //when RotMat(2,0) == -1
        {//Gimbal lock, pitch = 90deg
            rollangle = atan2(RotMat(0, 1), RotMat(0, 2));
        }
        else //general solution
        {
            rollangle = atan2(RotMat(2, 1), RotMat(2, 2));
        }

        return rollangle;
    }

    static double GetBodyYawAngle(Eigen::Matrix3d RotMat)
    {
        double pitchangle, yawangle;

        double threshold = 0.001;
        pitchangle = -asin(RotMat(2, 0));
        if (RotMat(2, 0) > 1.0 - threshold && RotMat(2, 0) < 1.0 + threshold) //when RotMat(2,0) == 1
        {//Gimbal lock, pitch = -90deg
            yawangle = 0.0;
        }
        else if (RotMat(2, 0) < -1.0 + threshold && RotMat(2, 0) > -1.0 - threshold) //when RotMat(2,0) == -1
        {//Gimbal lock, pitch = 90deg
            yawangle = 0.0;
        }
        else //general solution
        {
            yawangle = atan2(RotMat(1, 0), RotMat(0, 0));
        }

        return yawangle;
    }

    static Eigen::Matrix3d GetBodyRotationMatrix(double Roll, double Pitch, double Yaw)
    {
        Eigen::Matrix3d R_yaw;
        R_yaw.setZero();
        //yawŽÂ zÃà¿¡ ŽëÇÑ ÈžÀü
        R_yaw(2, 2) = 1.0;
        R_yaw(0, 0) = cos(Yaw);
        R_yaw(0, 1) = -sin(Yaw);
        R_yaw(1, 0) = sin(Yaw);
        R_yaw(1, 1) = cos(Yaw);

        Eigen::Matrix3d R_pitch;
        R_pitch.setZero();
        //pitchŽÂ yÃà¿¡ ŽëÇÑ ÈžÀü
        R_pitch(1, 1) = 1.0;
        R_pitch(0, 0) = cos(Pitch);
        R_pitch(2, 2) = cos(Pitch);
        R_pitch(0, 2) = sin(Pitch);
        R_pitch(2, 0) = -sin(Pitch);

        Eigen::Matrix3d R_roll;
        R_roll.setZero();
        //rollÀº xÃà¿¡ ŽëÇÑ ÈžÀü
        R_roll(0, 0) = 1.0;
        R_roll(1, 1) = cos(Roll);
        R_roll(2, 2) = cos(Roll);
        R_roll(1, 2) = -sin(Roll);
        R_roll(2, 1) = sin(Roll);

        Eigen::Matrix3d RGyro;
        RGyro.noalias() = R_yaw * R_pitch * R_roll;
        //dMatrix RGyro_inv(3,3);
        //RGyro.inv(RGyro_inv);

        //return RGyro_inv;
        return RGyro;
    }

    static Eigen::Matrix3d rotateWithZ(double yaw_angle)
    {
        Eigen::Matrix3d rotate_wth_z(3, 3);

        rotate_wth_z(0, 0) = cos(yaw_angle);
        rotate_wth_z(1, 0) = sin(yaw_angle);
        rotate_wth_z(2, 0) = 0.0;

        rotate_wth_z(0, 1) = -sin(yaw_angle);
        rotate_wth_z(1, 1) = cos(yaw_angle);
        rotate_wth_z(2, 1) = 0.0;

        rotate_wth_z(0, 2) = 0.0;
        rotate_wth_z(1, 2) = 0.0;
        rotate_wth_z(2, 2) = 1.0;

        return rotate_wth_z;
    }

    static Eigen::Matrix3d rotateWithY(double pitch_angle)
    {
        Eigen::Matrix3d rotate_wth_y(3, 3);

        rotate_wth_y(0, 0) = cos(pitch_angle);
        rotate_wth_y(1, 0) = 0.0;
        rotate_wth_y(2, 0) = -sin(pitch_angle);

        rotate_wth_y(0, 1) = 0.0;
        rotate_wth_y(1, 1) = 1.0;
        rotate_wth_y(2, 1) = 0.0;

        rotate_wth_y(0, 2) = sin(pitch_angle);
        rotate_wth_y(1, 2) = 0.0;
        rotate_wth_y(2, 2) = cos(pitch_angle);

        return rotate_wth_y;
    }

    static Eigen::Matrix3d rotateWithX(double roll_angle)
    {
        Eigen::Matrix3d rotate_wth_x(3, 3);

        rotate_wth_x(0, 0) = 1.0;
        rotate_wth_x(1, 0) = 0.0;
        rotate_wth_x(2, 0) = 0.0;

        rotate_wth_x(0, 1) = 0.0;
        rotate_wth_x(1, 1) = cos(roll_angle);
        rotate_wth_x(2, 1) = sin(roll_angle);

        rotate_wth_x(0, 2) = 0.0;
        rotate_wth_x(1, 2) = -sin(roll_angle);
        rotate_wth_x(2, 2) = cos(roll_angle);

        return rotate_wth_x;
    }

    static Eigen::Matrix3d skew(Eigen::Vector3d src)
    {
        Eigen::Matrix3d skew;
        skew.setZero();
        skew(0, 1) = -src[2];
        skew(0, 2) = src[1];
        skew(1, 0) = src[2];
        skew(1, 2) = -src[0];
        skew(2, 0) = -src[1];
        skew(2, 1) = src[0];

        return skew;
    }
    /*
    static Eigen::Vector3d getPhi(Eigen::Matrix3d current_rotation, Eigen::Matrix3d desired_rotation)
    {
        Eigen::Vector3d phi;
        Eigen::Vector3d s[3], v[3], w[3];
        for (int i = 0; i < 3; i++)
        {
            v[i] = current_rotation.block<3, 1>(0, i);
            w[i] = desired_rotation.block<3, 1>(0, i);
            s[i] = v[i].cross(w[i]);
        }
        phi = s[0] + s[1] + s[2];
        phi = -0.5 * phi;
        return phi;
    }
    */
    static Eigen::Vector3d getPhi(Eigen::Matrix3d& RotationMtx, Eigen::Matrix3d& DesiredRotationMtx) //Orientation 구성
    {
        //Get SkewSymmetric
        Eigen::Matrix3d s1_skew;
        Eigen::Matrix3d s2_skew;
        Eigen::Matrix3d s3_skew;

        Eigen::Vector3d RotMtxcol1;
        Eigen::Vector3d RotMtxcol2;
        Eigen::Vector3d RotMtxcol3;
        for (int i = 0; i < 3; i++)
        {
            RotMtxcol1(i) = RotationMtx(i, 0);
            RotMtxcol2(i) = RotationMtx(i, 1);
            RotMtxcol3(i) = RotationMtx(i, 2);
        }

        s1_skew = skew(RotMtxcol1);
        s2_skew = skew(RotMtxcol2);
        s3_skew = skew(RotMtxcol3);
        /////////////////////////////////////////////////////////////

        Eigen::Vector3d s1d;
        Eigen::Vector3d s2d;
        Eigen::Vector3d s3d;
        for (int i = 0; i < 3; i++)
        {
            s1d(i) = DesiredRotationMtx(i, 0);
            s2d(i) = DesiredRotationMtx(i, 1);
            s3d(i) = DesiredRotationMtx(i, 2);
        }

        Eigen::Vector3d s1f;
        Eigen::Vector3d s2f;
        Eigen::Vector3d s3f;

        s1f = s1_skew * s1d;
        s2f = s2_skew * s2d;
        s3f = s3_skew * s3d;
        /////////////////////////////////////////////////////////////
        //phi.resize(3);
        Eigen::Vector3d phi;
        phi = (s1f + s2f + s3f) * (-1.0 / 2.0);
        return phi;
    }

    static Eigen::Vector3d OrientationVelocity(Eigen::Matrix3d Rot, Eigen::Matrix3d Rotdot) //rotation matrix, derivative of rotation matrix
    {
        Eigen::Matrix3d RdotRT;
        RdotRT = Rotdot * Rot.transpose();
        Eigen::Vector3d OriVel;
        OriVel(0) = RdotRT(2, 1);
        OriVel(1) = RdotRT(0, 2);
        OriVel(2) = RdotRT(1, 0);

        return OriVel;
    }


    static double Cubic(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f)
    {
        double rx_q;

        if (rT < rT_0)
        {
            rx_q = rx_0;
        }
        else if (rT > rT_f)
        {
            rx_q = rx_f;
        }
        else {

            rx_q = rx_0 + rx_dot_0 * (rT - rT_0)
                + (3.0 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0)) - 2.0 * rx_dot_0 / (rT_f - rT_0) - rx_dot_f / (rT_f - rT_0)) * (rT - rT_0) * (rT - rT_0)
                + (-2.0 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0)) + (rx_dot_0 + rx_dot_f) / ((rT_f - rT_0) * (rT_f - rT_0))) * (rT - rT_0) * (rT - rT_0) * (rT - rT_0);
        }
        return (rx_q);
    }

    static double CubicDot(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f)
    {
        double rx_q_dot;

        if (rT < rT_0)
        {
            rx_q_dot = rx_dot_0;
        }
        else if (rT > rT_f)
        {
            rx_q_dot = rx_dot_f;
        }
        else {
            rx_q_dot = rx_dot_0 + 2.0 * (3.0 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0)) - 2.0 * rx_dot_0 / (rT_f - rT_0) - rx_dot_f / (rT_f - rT_0)) * (rT - rT_0)
                + 3.0 * (-2.0 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0)) + (rx_dot_0 + rx_dot_f) / ((rT_f - rT_0) * (rT_f - rT_0))) * (rT - rT_0) * (rT - rT_0);
        }
        return (rx_q_dot);
    }

    static double Min(double val1, double val2)
    {
        double min_val = 0.0;

        if (val1 >= val2)
        {
            min_val = val2;
        }
        else
        {
            min_val = val1;
        }

        return min_val;
    }

    static double Max(double val1, double val2)
    {
        double min_val = 0.0;

        if (val1 >= val2)
        {
            min_val = val1;
        }
        else
        {
            min_val = val2;
        }

        return min_val;
    }

    static double SwitchFunction(double start, double end, double val)
    {
        double alpha = 0.0;
        alpha = Cubic(val, start, end, 0.0, 0.0, 1.0, 0.0);
        return alpha;
    }

    static void ContactWrenchConeConstraintGenerate(double friction_coeff, double cop_x_b, double cop_y_b, double pushing_force_threshold, Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub) //A: constraint matrix
    {
        //This function is for contact wrench cone constraint
        //based on Caron, Stéphane, Quang-Cuong Pham, and Yoshihiko Nakamura. "Stability of surface contacts for humanoid robots: Closed-form formulae of the contact wrench cone for rectangular support areas." 2015 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2015.
        // * z direction denotes normal direction of contact plane (negative direction = toward object or ground)
        // * pushing_force_thrreshold should be negative number 
        // * coordinate has to locate center of the contact plane (cop_x_b = l_x/2, cop_y_b = l_y/2)
        // * friction_coeff, cop_x_b and cop_y_b should be positive number
        
        A.setZero(17, 6);
        lb.setZero(17);
        ub.setZero(17);

        double inf_val = 100000000.0; //large number to describe infinite

        // A matrix
        //CoP x
        A(0, 2) = -cop_x_b;
        A(0, 4) = -1.0;
        A(1, 2) = cop_x_b;
        A(1, 4) = -1.0;
        //CoP y
        A(2, 2) = -cop_y_b;
        A(2, 3) = 1.0;
        A(3, 2) = cop_y_b;
        A(3, 3) = 1.0;
        //Fx friction
        A(4, 0) = 1.0;
        A(4, 2) = -friction_coeff;
        A(5, 0) = 1.0;
        A(5, 2) = friction_coeff;
        //Fy friction
        A(6, 1) = 1.0;
        A(6, 2) = -friction_coeff;
        A(7, 1) = 1.0;
        A(7, 2) = friction_coeff;
        //Mz friction
        A(8, 0) = -cop_y_b;
        A(8, 1) = -cop_x_b;
        A(8, 2) = -friction_coeff*(cop_x_b+cop_y_b)/2.0;
        A(8, 3) = friction_coeff;
        A(8, 4) = friction_coeff;
        A(8, 5) = 1.0;
        A(9, 0) = -cop_y_b;
        A(9, 1) = cop_x_b;
        A(9, 2) = -friction_coeff * (cop_x_b + cop_y_b) / 2.0;
        A(9, 3) = friction_coeff;
        A(9, 4) = -friction_coeff;
        A(9, 5) = 1.0;
        A(10, 0) = cop_y_b;
        A(10, 1) = -cop_x_b;
        A(10, 2) = -friction_coeff * (cop_x_b + cop_y_b) / 2.0;
        A(10, 3) = -friction_coeff;
        A(10, 4) = friction_coeff;
        A(10, 5) = 1.0;
        A(11, 0) = cop_y_b;
        A(11, 1) = cop_x_b;
        A(11, 2) = -friction_coeff * (cop_x_b + cop_y_b) / 2.0;
        A(11, 3) = -friction_coeff;
        A(11, 4) = -friction_coeff;
        A(11, 5) = 1.0;
        A(12, 0) = -cop_y_b;
        A(12, 1) = -cop_x_b;
        A(12, 2) = -friction_coeff * (cop_x_b + cop_y_b) / 2.0;
        A(12, 3) = -friction_coeff;
        A(12, 4) = -friction_coeff;
        A(12, 5) = -1.0;
        A(13, 0) = -cop_y_b;
        A(13, 1) = cop_x_b;
        A(13, 2) = -friction_coeff * (cop_x_b + cop_y_b) / 2.0;
        A(13, 3) = -friction_coeff;
        A(13, 4) = friction_coeff;
        A(13, 5) = -1.0;
        A(14, 0) = cop_y_b;
        A(14, 1) = -cop_x_b;
        A(14, 2) = -friction_coeff * (cop_x_b + cop_y_b) / 2.0;
        A(14, 3) = friction_coeff;
        A(14, 4) = -friction_coeff;
        A(14, 5) = -1.0;
        A(15, 0) = cop_y_b;
        A(15, 1) = cop_x_b;
        A(15, 2) = -friction_coeff * (cop_x_b + cop_y_b) / 2.0;
        A(15, 3) = friction_coeff;
        A(15, 4) = friction_coeff;
        A(15, 5) = -1.0;
        //Fz (pushing force)
        A(16, 2) = 1.0;
        
        // lb vector        
        lb(1) = -inf_val;
        lb(3) = -inf_val;
        lb(5) = -inf_val;
        lb(7) = -inf_val;
        lb(16) = -inf_val;
        
        //ub vector
        ub(0) = inf_val;
        ub(2) = inf_val;
        ub(4) = inf_val;
        ub(6) = inf_val;
        ub(8) = inf_val;
        ub(9) = inf_val;
        ub(10) = inf_val;
        ub(11) = inf_val;
        ub(12) = inf_val;
        ub(13) = inf_val;
        ub(14) = inf_val;
        ub(15) = inf_val;
        ub(16) = pushing_force_threshold;
    }

    template<int Rows, int Cols> static void ContactWrenchConeConstraint(double mu, double cop_x_b, double cop_y_b,  Eigen::Matrix<double, Rows, Cols>& A,
    Eigen::Matrix<double, Rows, 1>& lb, Eigen::Matrix<double, Rows, 1>& ub)
    {    
        double inf_val = 10000.0; //large number to describe infinite

        Eigen::Matrix<double, 5, 6> Cop_mat = Eigen::Matrix<double, 5, 6>::Zero();
        Eigen::Matrix<double, 5, 1> Cop_lb = Eigen::Matrix<double, 5, 1>::Zero();
        Eigen::Matrix<double, 5, 1> Cop_ub = Eigen::Matrix<double, 5, 1>::Zero();
        Cop_mat(0,0) = -1.0;        Cop_mat(0,2) = -mu; // -fx <= -mu fz
        Cop_mat(1,0) =  1.0;        Cop_mat(1,2) = -mu; //  fx <= -mu fz
        Cop_mat(2,1) = -1.0;        Cop_mat(2,2) = -mu; // -fy <= -mu fz
        Cop_mat(3,1) =  1.0;        Cop_mat(3,2) = -mu; //  fy <= -mu fz
        Cop_mat(4,2) = -1.0;                            //  fz  >  0

        Cop_lb = -inf_val * Eigen::Matrix<double, 5, 1>::Ones();      

        // Eigen::Matrix<double, 4, 6> Cop_mat = Eigen::Matrix<double, 4, 6>::Zero();
        // Cop_mat(0,0) = -1.0;        Cop_mat(0,2) = -mu; // -fx <= -mu fz
        // Cop_mat(1,0) =  1.0;        Cop_mat(1,2) = -mu; //  fx <= -mu fz
        // Cop_mat(2,1) = -1.0;        Cop_mat(2,2) = -mu; // -fy <= -mu fz
        // Cop_mat(3,1) =  1.0;        Cop_mat(3,2) = -mu; //  fy <= -mu fz
        // // Cop_mat(4,2) = -1.0;                                        //  fz  >  0

        Eigen::Matrix<double, 4, 6> ZMP_mat = Eigen::Matrix<double, 4, 6>::Zero();
        Eigen::Matrix<double, 4, 1> ZMP_lb = Eigen::Matrix<double, 4, 1>::Zero();
        Eigen::Matrix<double, 4, 1> ZMP_ub = Eigen::Matrix<double, 4, 1>::Zero();
        ZMP_mat(0,2) = -cop_y_b;    ZMP_mat(0,3) = -1.0;
        ZMP_mat(1,2) = -cop_y_b;    ZMP_mat(1,3) =  1.0;
        ZMP_mat(2,2) = -cop_x_b;    ZMP_mat(2,4) = -1.0;
        ZMP_mat(3,2) = -cop_x_b;    ZMP_mat(3,4) =  1.0;

        ZMP_lb = -inf_val * Eigen::Matrix<double, 4, 1>::Ones();  

        Eigen::Matrix<double, 8, 6> non_yaw_mat = Eigen::Matrix<double, 8, 6>::Zero();
        Eigen::Matrix<double, 8, 1> non_yaw_lb = Eigen::Matrix<double, 8, 1>::Zero();
        Eigen::Matrix<double, 8, 1> non_yaw_ub = Eigen::Matrix<double, 8, 1>::Zero();
    
        non_yaw_mat(0,0) = -cop_y_b;    non_yaw_mat(0,1) = -cop_x_b;    non_yaw_mat(0,2) = -mu*(cop_x_b+cop_y_b);  non_yaw_mat(0,3) =  mu;    non_yaw_mat(0,4) =  mu;    non_yaw_mat(0,5) = -1.0;
        non_yaw_mat(1,0) = -cop_y_b;    non_yaw_mat(1,1) =  cop_x_b;    non_yaw_mat(1,2) = -mu*(cop_x_b+cop_y_b);  non_yaw_mat(1,3) =  mu;    non_yaw_mat(1,4) = -mu;    non_yaw_mat(1,5) = -1.0;
        non_yaw_mat(2,0) =  cop_y_b;    non_yaw_mat(2,1) = -cop_x_b;    non_yaw_mat(2,2) = -mu*(cop_x_b+cop_y_b);  non_yaw_mat(2,3) = -mu;    non_yaw_mat(2,4) =  mu;    non_yaw_mat(2,5) = -1.0;
        non_yaw_mat(3,0) =  cop_y_b;    non_yaw_mat(3,1) =  cop_x_b;    non_yaw_mat(3,2) = -mu*(cop_x_b+cop_y_b);  non_yaw_mat(3,3) = -mu;    non_yaw_mat(3,4) = -mu;    non_yaw_mat(3,5) = -1.0;
        non_yaw_mat(4,0) =  cop_y_b;    non_yaw_mat(4,1) =  cop_x_b;    non_yaw_mat(4,2) = -mu*(cop_x_b+cop_y_b);  non_yaw_mat(4,3) =  mu;    non_yaw_mat(4,4) =  mu;    non_yaw_mat(4,5) = 1.0;
        non_yaw_mat(5,0) =  cop_y_b;    non_yaw_mat(5,1) = -cop_x_b;    non_yaw_mat(5,2) = -mu*(cop_x_b+cop_y_b);  non_yaw_mat(5,3) =  mu;    non_yaw_mat(5,4) = -mu;    non_yaw_mat(5,5) = 1.0;
        non_yaw_mat(6,0) = -cop_y_b;    non_yaw_mat(6,1) =  cop_x_b;    non_yaw_mat(6,2) = -mu*(cop_x_b+cop_y_b);  non_yaw_mat(6,3) = -mu;    non_yaw_mat(6,4) =  mu;    non_yaw_mat(6,5) = 1.0;
        non_yaw_mat(7,0) = -cop_y_b;    non_yaw_mat(7,1) = -cop_x_b;    non_yaw_mat(7,2) = -mu*(cop_x_b+cop_y_b);  non_yaw_mat(7,3) = -mu;    non_yaw_mat(7,4) = -mu;    non_yaw_mat(7,5) = 1.0;

        // non_yaw_ub(0) = inf_val;        non_yaw_lb(4) = -inf_val;
        // non_yaw_ub(1) = inf_val;        non_yaw_lb(5) = -inf_val;
        // non_yaw_ub(2) = inf_val;        non_yaw_lb(6) = -inf_val;
        // non_yaw_ub(3) = inf_val;        non_yaw_lb(7) = -inf_val;

        non_yaw_lb = -inf_val * Eigen::Matrix<double, 8, 1>::Ones();  
        // std::cout << Cop_mat<< std::endl<<std::endl;
        // std::cout << ZMP_mat<< std::endl;
        A.setZero(Rows, Cols);

        A.block(0, 0, 5, 6) = Cop_mat;
        A.block(5, 0, 4, 6) = ZMP_mat;
        A.block(9, 0, 8, 6) = non_yaw_mat;

        lb.setZero(Rows);               ub.setZero(Rows);
        lb.segment(0,5) = Cop_lb;       ub.segment(0,5) = Cop_ub;
        lb.segment(5,4) = ZMP_lb;       ub.segment(5,4) = ZMP_ub;
        lb.segment(9,8) = non_yaw_lb;   ub.segment(9,8) = non_yaw_ub;

    }

    static void checkPositiveSemiDefinite(const Eigen::MatrixXd& H) {
        // std::cout << "Matrix H size: " << H.rows() << "x" << H.cols() << std::endl;
        // 고유값 계산
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(H);
        if (eigenSolver.info() != Eigen::Success) {
            // std::cerr << "Eigenvalue computation failed!" << std::endl;
            return;
        }

        // 고유값 출력
        // std::cout << "Eigenvalues of H:" << std::endl << eigenSolver.eigenvalues().transpose() << std::endl;

        // 양의 준정부호 여부 확인
        bool isPositiveSemiDefinite = true;
        for (int i = 0; i < eigenSolver.eigenvalues().size(); ++i) {
            if (eigenSolver.eigenvalues()[i] < -1e-6) { // 작은 음수는 수치적 오차로 간주
                isPositiveSemiDefinite = false;
                break;
            }
        }

        if (isPositiveSemiDefinite) {
            // std::cout << "Matrix H is Positive Semi-Definite." << std::endl;
        } else {
            std::cout << "Matrix H is NOT Positive Semi-Definite." << std::endl;
        }
    }

    static void checkRank(const Eigen::MatrixXd& A) {
        // std::cout << "Matrix A size: " << A.rows() << "x" << A.cols() << std::endl;
        // 랭크 계산
        Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(A);
        int rank = lu_decomp.rank();

        // 랭크 출력
        // std::cout << "Rank of A: " << rank << std::endl;

        // 행렬의 크기와 비교하여 선형 독립성 확인
        if (rank == A.rows()) {
            // std::cout << "Matrix A has full row rank (linearly independent constraints)." << std::endl;
        } else if (rank == A.cols()) {
            // std::cout << "Matrix A has full column rank (linearly independent variables)." << std::endl;
        } else {
            std::cout << "Rank of A: " << rank << std::endl;
            std::cout << "Matrix A does not have full rank (linearly dependent rows or columns)." << std::endl;
        }
    }

}


#endif