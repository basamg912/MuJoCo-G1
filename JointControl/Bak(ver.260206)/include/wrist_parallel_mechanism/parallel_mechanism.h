#ifndef PARALLEL_MECHANISM_H
#define PARALLEL_MECHANISM_H

#include <Eigen/Dense>
#include <iostream>

namespace parallel_mechanism {

    struct JointCoordinates {
        double Roll;
        double Pitch;
    };

    struct MotorCoordinates {
        double front_motor;
        double back_motor;
    };

    class ParallelMechanism {
    public:
        ParallelMechanism();
        ~ParallelMechanism();

        /**
         * @brief Calculates joint coordinates from motor coordinates.
         *        Updates the internal state (m_dCurrentRollL, m_dCurrentPitchL).
         * 
         * @param dFrontMotor Input motor 1 position
         * @param dBackMotor Input motor 2 position
         * @return JointCoordinates Struct containing Roll and Pitch
         */
        JointCoordinates ForwardKinematics_L(double dFrontMotor, double dBackMotor);
        JointCoordinates ForwardKinematics_R(double dFrontMotor, double dBackMotor);

        /**
         * @brief Calculates motor coordinates from joint coordinates.
         *        Updates the internal state (m_dCurrentRollL, m_dCurrentPitchL).
         * 
         * @param dJoint1 Input joint 1 position
         * @param dJoint2 Input joint 2 position
         * @return MotorCoordinates Struct containing front_motor and back_motor
         */
        MotorCoordinates InverseKinematics_L(double dJoint1, double dJoint2);
        MotorCoordinates InverseKinematics_R(double dJoint1, double dJoint2);

        /**
         * @brief Calculates the Jacobian matrix using the current internal state.
         * 
         * @return Eigen::Matrix2d The 2x2 Jacobian matrix
         */
        Eigen::Matrix2d Jacobian_L(double dFrontMotor, double dBackMotor);
        Eigen::Matrix2d Jacobian_R(double dFrontMotor, double dBackMotor);

        double getCurrentRoll_L() const { return m_dCurrentRollL; }
        double getCurrentPitch_L() const { return m_dCurrentPitchL; }
        double getCurrentRoll_R() const { return m_dCurrentRollR; }
        double getCurrentPitch_R() const { return m_dCurrentPitchR; }

        Eigen::Vector2d compute_linear_force_L(double dMotor1, double dMotor2, double dRollTorque, double dPitchTorque);
        Eigen::Vector2d compute_linear_force_R(double dMotor1, double dMotor2, double dRollTorque, double dPitchTorque);

    private:
        // Internal state
        double m_dCurrentRollL;
        double m_dCurrentRollR;
        double m_dCurrentPitchL;
        double m_dCurrentPitchR;

        // Geometric parameters
        double m_dL1, m_dL2, m_dL3, m_dL4, m_dL5, m_dL6;
        double m_dDx, m_dDy, m_dDz;
        
        // Constants
        const int m_nMaxIter = 20;
        const double m_dTolerance = 1e-10;

        // Helper methods
        void calculateResiduals_L(double dFrontMotor, double dBackMotor, double dJoint1, double dJoint2, double& dF1, double& dF2);
        void calculateResiduals_R(double dFrontMotor, double dBackMotor, double dJoint1, double dJoint2, double& dF1, double& dF2);
        void calculateJacobianInternal_L(double dFrontMotor, double dBackMotor, double dJoint1, double dJoint2, Eigen::Matrix2d& mJ);
        void calculateJacobianInternal_R(double dFrontMotor, double dBackMotor, double dJoint1, double dJoint2, Eigen::Matrix2d& mJ);
    };

}

#endif // PARALLEL_MECHANISM_H
