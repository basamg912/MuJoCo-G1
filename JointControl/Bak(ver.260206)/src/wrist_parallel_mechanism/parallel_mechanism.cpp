#include "wrist_parallel_mechanism/parallel_mechanism.h"

namespace parallel_mechanism {

    // Helper for Rotation Matrices (Internal linkage or static member)
    static Eigen::Matrix3d Rot_x(double dTheta) {
        double dC = std::cos(dTheta);
        double dS = std::sin(dTheta);
        Eigen::Matrix3d mR;
        mR << 1, 0, 0,
             0, dC, -dS,
             0, dS, dC;
        return mR;
    }

    static Eigen::Matrix3d Rot_y(double dTheta) {
        double dC = std::cos(dTheta);
        double dS = std::sin(dTheta);
        Eigen::Matrix3d mR;
        mR << dC, 0, dS,
             0, 1, 0,
             -dS, 0, dC;
        return mR;
    }

    static Eigen::Matrix3d dRot_x(double dTheta) {
        double dC = std::cos(dTheta);
        double dS = std::sin(dTheta);
        Eigen::Matrix3d mdR;
        mdR << 0, 0, 0,
              0, -dS, -dC,
              0, dC, -dS;
        return mdR;
    }

    static Eigen::Matrix3d dRot_y(double dTheta) {
        double dC = std::cos(dTheta);
        double dS = std::sin(dTheta);
        Eigen::Matrix3d mdR;
        mdR << -dS, 0, dC,
              0, 0, 0,
              -dC, 0, -dS;
        return mdR;
    }

    ParallelMechanism::ParallelMechanism() 
        : m_dCurrentRollL(0.0), m_dCurrentPitchL(0.0), m_dCurrentRollR(0.0), m_dCurrentPitchR(0.0)
    {
        // Initialize parameters (from Robot_Control.h)
        m_dL1 =  0.012000000;
        m_dL2 =  0.010500000;
        m_dL3 =  0.029690000;
        m_dL4 =  0.051000000;
        m_dL5 =  0.007300000;
        m_dL6 =  0.015000000;
        m_dDx =  0.009500000;
        m_dDy =  0.035470000;
        m_dDz =  0.031500000;
    }

    ParallelMechanism::~ParallelMechanism() {}

    void ParallelMechanism::calculateResiduals_L(double dFrontMotor, double dBackMotor, double dRoll, double dPitch, double& dF1, double& dF2) {
        // Fixed points (Motor attachment points)
        Eigen::Vector3d vPm1(m_dDx, -m_dDz, -dFrontMotor);
        Eigen::Vector3d vPm2(-m_dDx, -m_dDz, -dBackMotor);

        // Vectors
        Eigen::Vector3d vPBase(0, 0, -m_dDy);
        Eigen::Vector3d vL1(0, 0, -m_dL1);
        
        // Link 2 vectors
        Eigen::Vector3d vL2_1(m_dL6, -m_dL3, -m_dL2 + m_dL5);
        Eigen::Vector3d vL2_2(-m_dL6, -m_dL3, -m_dL2 + m_dL5);

        Eigen::Matrix3d mRx = Rot_x(dRoll);
        Eigen::Matrix3d mRy = Rot_y(dPitch);

        // Moving points
        Eigen::Vector3d vP1 = vPBase + mRx * vL1 + mRx * mRy * vL2_1;
        Eigen::Vector3d vP2 = vPBase + mRx * vL1 + mRx * mRy * vL2_2;

        // Residuals
        dF1 = (vP1 - vPm1).squaredNorm() - m_dL4 * m_dL4;
        dF2 = (vP2 - vPm2).squaredNorm() - m_dL4 * m_dL4;
    }

    void ParallelMechanism::calculateResiduals_R(double dFrontMotor, double dBackMotor, double dRoll, double dPitch, double& dF1, double& dF2) {
        // Fixed points (Motor attachment points)
        Eigen::Vector3d vPm1( m_dDx, -m_dDz, dFrontMotor);
        Eigen::Vector3d vPm2(-m_dDx, -m_dDz, dBackMotor);

        // Vectors
        Eigen::Vector3d vPBase(0, 0, m_dDy);
        Eigen::Vector3d vL1(0, 0, m_dL1);
        
        // Link 2 vectors
        Eigen::Vector3d vL2_1( m_dL6, -m_dL3, m_dL2 - m_dL5);
        Eigen::Vector3d vL2_2(-m_dL6, -m_dL3, m_dL2 - m_dL5);

        Eigen::Matrix3d mRx = Rot_x(dRoll);
        Eigen::Matrix3d mRy = Rot_y(dPitch);

        // Moving points
        Eigen::Vector3d vP1 = vPBase + mRx * vL1 + mRx * mRy * vL2_1;
        Eigen::Vector3d vP2 = vPBase + mRx * vL1 + mRx * mRy * vL2_2;

        // Residuals
        dF1 = (vP1 - vPm1).squaredNorm() - m_dL4 * m_dL4;
        dF2 = (vP2 - vPm2).squaredNorm() - m_dL4 * m_dL4;
    }

    void ParallelMechanism::calculateJacobianInternal_L(double dFrontMotor, double dBackMotor, double dRoll, double dPitch, Eigen::Matrix2d& mJ) {
        Eigen::Vector3d vPm1(m_dDx, -m_dDz, -dFrontMotor);
        Eigen::Vector3d vPm2(-m_dDx, -m_dDz, -dBackMotor);

        Eigen::Vector3d vPBase(0, 0, -m_dDy);
        Eigen::Vector3d vL1(0, 0, -m_dL1);
        Eigen::Vector3d vL2_1(m_dL6, -m_dL3, -m_dL2 + m_dL5);
        Eigen::Vector3d vL2_2(-m_dL6, -m_dL3, -m_dL2 + m_dL5);

        Eigen::Matrix3d mRx = Rot_x(dRoll);
        Eigen::Matrix3d mRy = Rot_y(dPitch);
        Eigen::Matrix3d mdRx = dRot_x(dRoll);
        Eigen::Matrix3d mdRy = dRot_y(dPitch);

        Eigen::Vector3d vP1 = vPBase + mRx * vL1 + mRx * mRy * vL2_1;
        Eigen::Vector3d vP2 = vPBase + mRx * vL1 + mRx * mRy * vL2_2;

        Eigen::Vector3d vdP1_dj1 = mdRx * vL1 + mdRx * mRy * vL2_1;
        Eigen::Vector3d vdP1_dj2 = mRx * mdRy * vL2_1;

        Eigen::Vector3d vdP2_dj1 = mdRx * vL1 + mdRx * mRy * vL2_2;
        Eigen::Vector3d vdP2_dj2 = mRx * mdRy * vL2_2;

        mJ(0, 0) = 2.0 * (vP1 - vPm1).dot(vdP1_dj1);
        mJ(0, 1) = 2.0 * (vP1 - vPm1).dot(vdP1_dj2);
        mJ(1, 0) = 2.0 * (vP2 - vPm2).dot(vdP2_dj1);
        mJ(1, 1) = 2.0 * (vP2 - vPm2).dot(vdP2_dj2);
    }

    void ParallelMechanism::calculateJacobianInternal_R(double dFrontMotor, double dBackMotor, double dRoll, double dPitch, Eigen::Matrix2d& mJ) {
        Eigen::Vector3d vPm1( m_dDx, -m_dDz, dFrontMotor);
        Eigen::Vector3d vPm2(-m_dDx, -m_dDz, dBackMotor);

        Eigen::Vector3d vPBase(0, 0, m_dDy);
        Eigen::Vector3d vL1(0, 0, m_dL1);
        Eigen::Vector3d vL2_1( m_dL6, -m_dL3, m_dL2 - m_dL5);
        Eigen::Vector3d vL2_2(-m_dL6, -m_dL3, m_dL2 - m_dL5);

        Eigen::Matrix3d mRx = Rot_x(dRoll);
        Eigen::Matrix3d mRy = Rot_y(dPitch);
        Eigen::Matrix3d mdRx = dRot_x(dRoll);
        Eigen::Matrix3d mdRy = dRot_y(dPitch);

        Eigen::Vector3d vP1 = vPBase + mRx * vL1 + mRx * mRy * vL2_1;
        Eigen::Vector3d vP2 = vPBase + mRx * vL1 + mRx * mRy * vL2_2;

        Eigen::Vector3d vdP1_dj1 = mdRx * vL1 + mdRx * mRy * vL2_1;
        Eigen::Vector3d vdP1_dj2 = mRx * mdRy * vL2_1;

        Eigen::Vector3d vdP2_dj1 = mdRx * vL1 + mdRx * mRy * vL2_2;
        Eigen::Vector3d vdP2_dj2 = mRx * mdRy * vL2_2;

        mJ(0, 0) = 2.0 * (vP1 - vPm1).dot(vdP1_dj1);
        mJ(0, 1) = 2.0 * (vP1 - vPm1).dot(vdP1_dj2);
        mJ(1, 0) = 2.0 * (vP2 - vPm2).dot(vdP2_dj1);
        mJ(1, 1) = 2.0 * (vP2 - vPm2).dot(vdP2_dj2);
    }

    JointCoordinates ParallelMechanism::ForwardKinematics_L(double dFrontMotor, double dBackMotor) {
        double dRoll = m_dCurrentRollL; // Start from current state
        double dPitch = m_dCurrentPitchL;
        
        for (int i = 0; i < m_nMaxIter; ++i) {
            double dF1, dF2;
            calculateResiduals_L(dFrontMotor, dBackMotor, dRoll, dPitch, dF1, dF2);

            Eigen::Vector2d vF;
            vF << dF1, dF2;

            if (std::abs(dF1) < m_dTolerance && std::abs(dF2) < m_dTolerance) {
                break;
            }

            Eigen::Matrix2d mJ;
            calculateJacobianInternal_L(dFrontMotor, dBackMotor, dRoll, dPitch, mJ);

            if (std::abs(mJ.determinant()) < 1e-9) {
                std::cerr << "Singularity in ForwardKinematics!" << std::endl;
                break;
            }
            Eigen::Vector2d vDelta = -mJ.inverse() * vF;

            dRoll += vDelta(0);
            dPitch += vDelta(1);
        }

        // Update state
        m_dCurrentRollL = dRoll;
        m_dCurrentPitchL = dPitch;

        return {dRoll, dPitch};
    }

    JointCoordinates ParallelMechanism::ForwardKinematics_R(double dFrontMotor, double dBackMotor) {
        double dRoll = m_dCurrentRollR; // Start from current state
        double dPitch = m_dCurrentPitchR;
        
        for (int i = 0; i < m_nMaxIter; ++i) {
            double dF1, dF2;
            calculateResiduals_R(dFrontMotor, dBackMotor, dRoll, dPitch, dF1, dF2);

            Eigen::Vector2d vF;
            vF << dF1, dF2;

            if (std::abs(dF1) < m_dTolerance && std::abs(dF2) < m_dTolerance) {
                break;
            }

            Eigen::Matrix2d mJ;
            calculateJacobianInternal_R(dFrontMotor, dBackMotor, dRoll, dPitch, mJ);

            if (std::abs(mJ.determinant()) < 1e-9) {
                std::cerr << "Singularity in ForwardKinematics!" << std::endl;
                break;
            }
            Eigen::Vector2d vDelta = -mJ.inverse() * vF;

            dRoll += vDelta(0);
            dPitch += vDelta(1);
        }

        // Update state
        m_dCurrentRollR = dRoll;
        m_dCurrentPitchR = dPitch;

        return {dRoll, dPitch};
    }

    MotorCoordinates ParallelMechanism::InverseKinematics_L(double dRoll, double dPitch) {
        // Update state
        m_dCurrentRollL = dRoll;
        m_dCurrentPitchL = dPitch;
        
        Eigen::Vector3d vPBase(0, 0, -m_dDy);
        Eigen::Vector3d vL1(0, 0, -m_dL1);
        Eigen::Vector3d vL2_1(m_dL6, -m_dL3, -m_dL2 + m_dL5);
        Eigen::Vector3d vL2_2(-m_dL6, -m_dL3, -m_dL2 + m_dL5);

        Eigen::Matrix3d mRx = Rot_x(dRoll);
        Eigen::Matrix3d mRy = Rot_y(dPitch);

        Eigen::Vector3d vP1 = vPBase + mRx * vL1 + mRx * mRy * vL2_1;
        Eigen::Vector3d vP2 = vPBase + mRx * vL1 + mRx * mRy * vL2_2;

        // For motor 1: Pm1 = [dx, -dz, m1]
        double dTerm1 = m_dL4*m_dL4 - std::pow(vP1.x() - m_dDx, 2) - std::pow(vP1.y() + m_dDz, 2);
        double dM1 = 0.0;
        if (dTerm1 >= 0) {
             dM1 = vP1.z() + std::sqrt(dTerm1);
        } else {
            std::cout << "Unreachable workspace in InverseKinematics (Motor 1)" << std::endl;
        }

        // For motor 2: Pm2 = [-dx, -dz, m2]
        double dTerm2 = m_dL4*m_dL4 - std::pow(vP2.x() + m_dDx, 2) - std::pow(vP2.y() + m_dDz, 2);
        double dM2 = 0.0;
        if (dTerm2 >= 0) {
             dM2 = vP2.z() + std::sqrt(dTerm2);
        } else {
            std::cout << "Unreachable workspace in InverseKinematics (Motor 2)" << std::endl;
        }

        return {-dM1, -dM2};
    }

    MotorCoordinates ParallelMechanism::InverseKinematics_R(double dRoll, double dPitch) {
        // Update state
        m_dCurrentRollR = dRoll;
        m_dCurrentPitchR = dPitch;
        
        Eigen::Vector3d vPBase(0, 0, m_dDy);
        Eigen::Vector3d vL1(0, 0, m_dL1);
        Eigen::Vector3d vL2_1( m_dL6, -m_dL3, m_dL2 - m_dL5);
        Eigen::Vector3d vL2_2(-m_dL6, -m_dL3, m_dL2 - m_dL5);

        Eigen::Matrix3d mRx = Rot_x(dRoll);
        Eigen::Matrix3d mRy = Rot_y(dPitch);

        Eigen::Vector3d vP1 = vPBase + mRx * vL1 + mRx * mRy * vL2_1;
        Eigen::Vector3d vP2 = vPBase + mRx * vL1 + mRx * mRy * vL2_2;

        // For motor 1: Pm1 = [dx, -dz, m1]
        double dTerm1 = m_dL4*m_dL4 - std::pow(vP1.x() - m_dDx, 2) - std::pow(vP1.y() + m_dDz, 2);
        double dM1 = 0.0;
        if (dTerm1 >= 0) {
             dM1 = vP1.z() - std::sqrt(dTerm1);
        } else {
            std::cout << "Unreachable workspace in InverseKinematics (Motor 1)" << std::endl;
        }

        // For motor 2: Pm2 = [-dx, -dz, m2]
        double dTerm2 = m_dL4*m_dL4 - std::pow(vP2.x() + m_dDx, 2) - std::pow(vP2.y() + m_dDz, 2);
        double dM2 = 0.0;
        if (dTerm2 >= 0) {
             dM2 = vP2.z() - std::sqrt(dTerm2);
        } else {
            std::cout << "Unreachable workspace in InverseKinematics (Motor 2)" << std::endl;
        }

        return {dM1, dM2};
    }

    Eigen::Matrix2d ParallelMechanism::Jacobian_L(double dFrontMotor, double dBackMotor) {
        // 1. Get current joint angles
        JointCoordinates sJoints = ForwardKinematics_L(dFrontMotor, dBackMotor);
        double dQ1 = sJoints.Roll;
        double dQ2 = sJoints.Pitch;

        // 2. Define vectors (same as calculateResiduals)
        Eigen::Vector3d vPm1(m_dDx, -m_dDz, -dFrontMotor);
        Eigen::Vector3d vPm2(-m_dDx, -m_dDz, -dBackMotor);
        Eigen::Vector3d vPBase(0, 0, -m_dDy);
        Eigen::Vector3d vL1(0, 0, -m_dL1);
        Eigen::Vector3d vL2_1(m_dL6, -m_dL3, -m_dL2 + m_dL5);
        Eigen::Vector3d vL2_2(-m_dL6, -m_dL3, -m_dL2 + m_dL5);

        // 3. Rotation matrices and derivatives
        Eigen::Matrix3d mRx = Rot_x(dQ1);
        Eigen::Matrix3d mRy = Rot_y(dQ2);
        Eigen::Matrix3d mdRx = dRot_x(dQ1);
        Eigen::Matrix3d mdRy = dRot_y(dQ2);

        // 4. Calculate P1, P2
        Eigen::Vector3d vP1 = vPBase + mRx * vL1 + mRx * mRy * vL2_1;
        Eigen::Vector3d vP2 = vPBase + mRx * vL1 + mRx * mRy * vL2_2;

        // 5. Calculate derivatives of P1, P2 w.r.t q1, q2
        Eigen::Vector3d vdP1_dq1 = mdRx * vL1 + mdRx * mRy * vL2_1;
        Eigen::Vector3d vdP1_dq2 = mRx * mdRy * vL2_1;

        Eigen::Vector3d vdP2_dq1 = mdRx * vL1 + mdRx * mRy * vL2_2;
        Eigen::Vector3d vdP2_dq2 = mRx * mdRy * vL2_2;

        Eigen::Vector3d vdPm1_m1 = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d vdPm2_m2 = Eigen::Vector3d::UnitZ();

        // 6. Construct A Matrix (df/dq)
        Eigen::Matrix2d mA;
        mA(0, 0) = 2.0 * (vP1 - vPm1).dot(vdP1_dq1); // df1/dq1
        mA(0, 1) = 2.0 * (vP1 - vPm1).dot(vdP1_dq2); // df1/dq2
        mA(1, 0) = 2.0 * (vP2 - vPm2).dot(vdP2_dq1); // df2/dq1
        mA(1, 1) = 2.0 * (vP2 - vPm2).dot(vdP2_dq2); // df2/dq2

        // 7. Construct B Matrix (df/dm)
        Eigen::Matrix2d mB;
        mB.setZero();
        mB(0, 0) = -2.0 * (vP1 - vPm1).dot(vdPm1_m1);
        mB(1, 1) = -2.0 * (vP2 - vPm2).dot(vdPm2_m2);

        // 8. Calculate Jacobian J = -A^-1 * B
        return -mA.inverse() * mB;
    }

    Eigen::Matrix2d ParallelMechanism::Jacobian_R(double dFrontMotor, double dBackMotor) {
        // 1. Get current joint angles
        JointCoordinates sJoints = ForwardKinematics_R(dFrontMotor, dBackMotor);
        double dQ1 = sJoints.Roll;
        double dQ2 = sJoints.Pitch;

        // 2. Define vectors (same as calculateResiduals)
        Eigen::Vector3d vPm1( m_dDx, -m_dDz, dFrontMotor);
        Eigen::Vector3d vPm2(-m_dDx, -m_dDz, dBackMotor);
        Eigen::Vector3d vPBase(0, 0, m_dDy);
        Eigen::Vector3d vL1(0, 0, m_dL1);
        Eigen::Vector3d vL2_1( m_dL6, -m_dL3, m_dL2 - m_dL5);
        Eigen::Vector3d vL2_2(-m_dL6, -m_dL3, m_dL2 - m_dL5);

        // 3. Rotation matrices and derivatives
        Eigen::Matrix3d mRx = Rot_x(dQ1);
        Eigen::Matrix3d mRy = Rot_y(dQ2);
        Eigen::Matrix3d mdRx = dRot_x(dQ1);
        Eigen::Matrix3d mdRy = dRot_y(dQ2);

        // 4. Calculate P1, P2
        Eigen::Vector3d vP1 = vPBase + mRx * vL1 + mRx * mRy * vL2_1;
        Eigen::Vector3d vP2 = vPBase + mRx * vL1 + mRx * mRy * vL2_2;

        // 5. Calculate derivatives of P1, P2 w.r.t q1, q2
        Eigen::Vector3d vdP1_dq1 = mdRx * vL1 + mdRx * mRy * vL2_1;
        Eigen::Vector3d vdP1_dq2 = mRx * mdRy * vL2_1;

        Eigen::Vector3d vdP2_dq1 = mdRx * vL1 + mdRx * mRy * vL2_2;
        Eigen::Vector3d vdP2_dq2 = mRx * mdRy * vL2_2;

        Eigen::Vector3d vdPm1_m1 = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d vdPm2_m2 = Eigen::Vector3d::UnitZ();

        // 6. Construct A Matrix (df/dq)
        Eigen::Matrix2d mA;
        mA(0, 0) = 2.0 * (vP1 - vPm1).dot(vdP1_dq1); // df1/dq1
        mA(0, 1) = 2.0 * (vP1 - vPm1).dot(vdP1_dq2); // df1/dq2
        mA(1, 0) = 2.0 * (vP2 - vPm2).dot(vdP2_dq1); // df2/dq1
        mA(1, 1) = 2.0 * (vP2 - vPm2).dot(vdP2_dq2); // df2/dq2

        // 7. Construct B Matrix (df/dm)
        Eigen::Matrix2d mB;
        mB.setZero();
        mB(0, 0) = -2.0 * (vP1 - vPm1).dot(vdPm1_m1);
        mB(1, 1) = -2.0 * (vP2 - vPm2).dot(vdPm2_m2);

        // 8. Calculate Jacobian J = -A^-1 * B
        return -mA.inverse() * mB;
    }

    Eigen::Vector2d ParallelMechanism::compute_linear_force_L(double dMotor1, double dMotor2, double dRollTorque, double dPitchTorque) {
        Eigen::Vector2d vRollPitchTorque;
        vRollPitchTorque << dRollTorque, dPitchTorque;

        return (Jacobian_L(dMotor1, dMotor2).inverse()).transpose() * vRollPitchTorque;
    }

    Eigen::Vector2d ParallelMechanism::compute_linear_force_R(double dMotor1, double dMotor2, double dRollTorque, double dPitchTorque) {
        Eigen::Vector2d vRollPitchTorque;
        vRollPitchTorque << dRollTorque, dPitchTorque;

        return (Jacobian_R(dMotor1, dMotor2).inverse()).transpose() * vRollPitchTorque;
    }

}