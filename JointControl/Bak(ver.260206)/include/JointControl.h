#ifndef __JOINT_CONTROL__
#define __JOINT_CONTROL__

#include <iostream>
#include <Eigen/Dense>
// #include <rbdl/rbdl.h>
#include <fstream>
#include "custommath.h"
#include <mutex>
#include <thread>
#include "networks/MLP.h"
#include "robotmodel.h"
#include "trajectory.h"
#include <random>

#include "posix_rt.h"

#include "StateDef.h"
#include "RLControl.h"
#include "MimicRLControl.h"
#include "wrist_parallel_mechanism/parallel_mechanism.h"

enum JOINT {
  L_shoulder_pitch = 19,
  L_shoulder_roll = 20,
  L_shoulder_yaw = 21,
  L_elbow = 22,
  L_wrist_yaw = 23,
  L_wrist_roll = 24,
  L_wrist_pitch = 25,

  R_shoulder_pitch = 26,
  R_shoulder_roll = 27,
  R_shoulder_yaw = 28,
  R_elbow = 29,
  R_wrist_yaw = 30,
  R_wrist_roll = 31,
  R_wrist_pitch = 32,
};

enum ABS_ENCODER {
  L_wrist_pitch_abs = 23,
  L_wrist_roll_abs = 24,

  R_wrist_pitch_abs = 30,
  R_wrist_roll_abs = 31,
};

enum MOTOR_DRIVER {
  L_shoulder_pitch_D = 19,
  L_shoulder_roll_D = 20,
  L_shoulder_yaw_D = 21,
  L_elbow_D = 22,
  L_wrist_front_D = 23,
  L_wrist_back_D = 24,
  L_wrist_yaw_D = 25,

  R_shoulder_pitch_D = 26,
  R_shoulder_roll_D = 27,
  R_shoulder_yaw_D = 28,
  R_elbow_D = 29,
  R_wrist_front_D = 30,
  R_wrist_back_D = 31,
  R_wrist_yaw_D = 32,
};

class CJointControl
{
public:
    CJointControl();
    virtual ~CJointControl();

    //==============================================================================
    // Public Functions
    //==============================================================================
    /**
     * @brief Converts motor state (position and velocity) to joint state (position and velocity).
     * 
     * This function takes the motor positions and velocities as input, along with the current time,
     * and performs the following operations:
     * 1. Saves the motor positions and velocities into internal member variables.
     * 2. Updates the real-time variable.
     * 3. Converts the motor positions to joint positions using the psi2q function.
     * 4. Saves the initial joint position during the first loop iteration.
     * 5. Computes the Actuation Jacobian matrix and uses it to calculate joint velocities.
     * 6. Optionally applies a low-pass filter (LPF) to the joint velocities (currently commented out).
     * 7. Updates the joint velocities with either the raw or filtered values.
     * 
     * @param Time Current time in seconds.
     * @param dTheta Array of motor positions (in radians).
     * @param dThetaDot Array of motor velocities (in radians per second).
     */
    void ConvertStateMotor2Joint(uint8_t task, int nState, double dTime, double *dTheta, double *dThetaDot, double *dThetaAbs, double dNeckPitch, double dNeckYaw);
    /**
     * @brief Converts the calculated joint torques to motor target commands.
     * 
     * This function takes the computed torque vector `m_torque` and assigns its values
     * to the `dTarget` array, which is sent to the motors. It also includes a safety
     * feature that disables torques if they exceed predefined limits, preventing potential
     * hardware damage.
     * 
     * @param dTarget Pointer to an array where the target motor torques will be stored.
     */
    void ConvertCmdJoint2Motor(double *dTarget);  

    
    
    const Vector<double, NUM_OF_JOINTS>& GetJointQrad() const { return m_vJointQrad; } 
    
    const Vector<double, NUM_OF_JOINTS>& GetJointQdotrad() const { return m_vJointQdotrad; }     

    void SetDesiredPosition(const std::array<float, 17>& qDes);

    //==============================================================================
    // Public Robot State
    //==============================================================================
	Vector<double, NUM_OF_JOINTS> m_vTorque; 
    Vector<double, NUM_OF_JOINTS> m_vMotorThetarad, m_vMotorThetaradDes, m_vPreMotorThetaradDes;
    Vector<double, NUM_OF_JOINTS> m_vMotorThetadotrad, m_vMotorThetadotradDes, m_vPrMotorThetadotradDes;
    Vector<double, NUM_OF_JOINTS> m_vMotorThetaAbsrad;
    Vector<double, NUM_OF_JOINTS> m_vJointQrad, m_vJointQradDes, m_vPreJointQradDes, m_vPreJointQrad;
    Vector<double, NUM_OF_JOINTS> m_vJointQdotrad, m_vJointQdotradDes, m_vPreJointQdotradDes;

	Vector<double, NUM_OF_JOINTS> m_vTorqueLPF, m_vTorqueLPF_Pre;
	Vector<double, NUM_OF_JOINTS> m_vRLTorque;
    Vector<double, NUM_OF_JOINTS> m_vRLJointQradDes, m_vRLJointQdotradDes;
    Vector<double, NUM_OF_JOINTS> m_vRLJointQradDesLPF_Pre;
    Vector<double, NUM_OF_JOINTS> m_vRLJactions, m_vRLJobservations;
    
    int m_iControlMode, m_iPreviousControlMode;

    Vector<double, 4> m_vWristLinearAbs;


    //==============================================================================
    // public model update
    //==============================================================================
    Vector<double, 6> m_vX_left_heel, m_vX_left_toe;
    Vector<double, 6> m_vXdes_left_heel, m_vXdes_left_toe;
    Vector<double, 6> m_vXdot_left_heel, m_vXdot_left_toe;
    Vector<double, 6> m_vXdotdes_left_heel, m_vXdotdes_left_toe;



    //==============================================================================
    // Public Control Gains
    //==============================================================================
	Eigen::Matrix<double, NUM_OF_JOINTS, NUM_OF_JOINTS> m_mKpj, m_mKdj; //joint control PD gain
	Eigen::Matrix<double, NUM_OF_JOINTS, NUM_OF_JOINTS> m_mKpRLj, m_mKdRLj; //RL joint control PD gain



    //==============================================================================
    // public debug state
    //==============================================================================
    Vector<double, 5> m_vComputationTime;
    bool m_bTorqueOff;

private:
    // Motion Retargeting
    // 궤적 미리 계산
    void precompute_trajectory();

    void load_segmented_data(const std::string& name_path, 
                             const std::string& theta_path, 
                             const std::string& vel_path,
                             const std::vector<std::string>& target_joints);
    std::vector<std::string> read_csv_names(const std::string& path);
    void parse_segmented_csv(const std::string& path, 
                             const std::vector<int>& target_indices, 
                             std::vector<Eigen::VectorXd>& buffer);

    // Trajectory Data
    std::vector<Eigen::VectorXd> data_q_buffer_;   // 50Hz q 데이터 저장
    std::vector<Eigen::VectorXd> data_vel_buffer_; // 50Hz qdot 데이터 저장
    
    int playback_idx_;      // 현재 재생 중인 50Hz 데이터 인덱스
    double playback_time_;  // 현재 세그먼트(0.02초) 내에서의 로컬 시간

    double csv_dt_ = 0.0333;
    int current_csv_idx_ = -1;
    
    CTrajectory joint_traj_;

    std::vector<Eigen::VectorXd> q_trajectory_;     // CSV Raw Data
    std::vector<Eigen::VectorXd> q_dot_trajectory_; // CSV Raw Data
    std::vector<int> target_indices_;
    std::vector<int> robot_q_indices_;

    std::vector<Eigen::VectorXd> q_interp_buffer_;
    std::vector<Eigen::VectorXd> q_dot_interp_buffer_;
    int playback_tick_ = 0; // 버퍼 재생 인덱스

    const std::string PATH_NAMES = "../utility/ref_motion/seg_thumbsup_kapex_isaac_source_v2_joint_names.csv";
    const std::string PATH_THETA = "../utility/ref_motion/seg_thumbsup_kapex_isaac_source_v2_joint_theta.csv";
    const std::string PATH_VEL   = "../utility/ref_motion/seg_thumbsup_kapex_isaac_source_v2_joint_vel.csv";

    const std::vector<std::string> ACTIVE_JOINT_INDEX = {
        "WLJ1", "WLJ2", "WLJ3",                                         // Waist (3)
        "HLJ1", "HLJ2",                                                 // Head (2)
        "LAJ1", "LAJ2", "LAJ3", "LAJ4", "LAJ5", "LAJ6", "LAJ7",         // Left Arm (7)
        "RAJ1", "RAJ2", "RAJ3", "RAJ4", "RAJ5", "RAJ6", "RAJ7"          // Right Arm (7)
    };

    const std::vector<int> TARGET_ROBOT_INDICES = {
        14, 15, 16,                             // Waist
        17, 18,                                 // Head
        19, 20, 21, 22, 23, 24, 25,             // Left Arm
        26, 27, 28, 29, 30, 31, 32              // Right Arm
    };

	// RLControl *m_pcRLControl = NULL;

    //==============================================================================
    // Private Functions
    //==============================================================================
    void Initialize();
    /**
     * @brief Updates the robot's dynamic and kinematic model.
     * 
     * This function uses the current joint positions (m_dJointQrad) and velocities (m_dJointQdotrad)
     * to update the robot's internal model. It recalculates various properties, including:
     * - Kinematics and dynamics of the robot.
     * - Jacobians for the end-effectors (heel and toe).
     * - Positions, orientations, and velocities of the end-effectors.
     * The updated model information is then stored in member variables for use in control calculations.
     */
    void ModelUpdate();

    /**
     * @brief Computes the control torque based on the selected control mode.
     * 
     * This function is the core of the control logic. It calculates the required motor torques
     * based on the value of `m_ControlMode`:
     * - Mode 1: Joint-space PD control. It follows a predefined sinusoidal trajectory for tuning gains.
     * - Mode 2: Task-space control using Closed-Loop Inverse Kinematics (CLIK). It tracks a desired
     *   end-effector trajectory.
     * - Mode 3: Model Predictive Path Integral (MPPI) control. It optimizes motor commands to follow
     *   a desired trajectory while considering system dynamics and constraints.
     * The final computed torque is stored in the `m_torque` member variable.
     */
    void Compute();

    int m_nControlMode;//1:Joint, 2.CLIK, 3:MPPI
    CModelKAPEX m_modelRobot;



    //==============================================================================
    // Robot Internal State
    //==============================================================================
    const int m_nJDoF = NUM_OF_JOINTS; //set real joint dof
    double m_dTime, m_dTimePrev; //current time
    double m_dInitStartTime, m_dInitTime; // init time
    double m_dHomingStartTime, m_dHomingTime; // homing time
    double m_dControlStartTime, m_dControlTime; // control time
    double m_dRLStartTime, m_dRLTime; // RL time
    bool m_bHOMING, m_bCONTROL, m_bMotion, m_bRL, m_bFirstLoop, m_bINIT, m_bINITCOMPLETE, m_bInitWrist;

    //==============================================================================
    // Motor to Joint Kinematics
    //==============================================================================
    /**
     * @brief Converts motor angles (psi) to joint angles (q).
     * 
     * This function implements the forward kinematics from the motor space to the joint space.
     * It involves several steps:
     * 1. Direct mapping for the hip joints (q0, q1, q2).
     * 2. Geometric calculation for the knee joint (q3) based on a four-bar linkage mechanism.
     * 3. A Multi-Layer Perceptron (MLP) neural network is used to compute the ankle pitch and roll
     *    joint angles (q4, q5) from the corresponding motor inputs (psi4, psi5).
     * 4. Geometric calculation for the toe joint (q6), which depends on the ankle pitch angle (q4).
     * 
     * @param psi A 7-element vector of motor angles.
     * @return A 7-element vector of the corresponding joint angles.
     */
    Vector<double, NUM_OF_JOINTS> psi2q(Vector<double, NUM_OF_JOINTS> vPsi);

    Eigen::Vector<double, 4> compute_wrist_q2psi(Vector<double,2> vLeftwristq, Vector<double,2> vRightwristq);

    /**
     * @brief Computes the Actuation Jacobian matrix.
     * 
     * This function calculates the Jacobian that maps motor velocities to joint velocities (J = dq/dpsi).
     * It uses numerical differentiation (central difference method) to find the partial derivatives
     * of the joint angles with respect to the motor angles. This is necessary because the `psi2q`
     * conversion is complex and involves an MLP, making analytical differentiation difficult.
     * The resulting Jacobian is used to transform motor velocities to joint velocities and to
     * transform joint torques to motor torques.
     * 
     * @param x The current vector of motor angles (psi).
     * @return The 14x14 Actuation Jacobian matrix.
     */
    Eigen::Matrix<double, NUM_OF_JOINTS, NUM_OF_JOINTS> compute_tau_q2psi(Vector<double,NUM_OF_JOINTS> vX);

	Eigen::Matrix<double, NUM_OF_JOINTS, NUM_OF_JOINTS> m_mActuation_tau_q2psi;
    Vector<double, NUM_OF_JOINTS> m_vJpsidot, m_vJpsidot_filtered;


    //==============================================================================
    // MLP state
    //==============================================================================
	MLPs m_mlp;



    //==============================================================================
    // Model-related Variables
    //==============================================================================
    bool m_bFisrtModelUpdate;
    Eigen::Matrix<double, 6, 6> m_mJ_left_heel;
    Eigen::Matrix<double, 6, 7> m_mJ_left_toe;




    //==============================================================================
    // Joint Control
    //==============================================================================
    bool m_bjointmotion, m_bmotormotion;
	CTrajectory JointTrajectory, MotorTrajectory;
    double m_dmotiontime, m_dmotionendtime;
    Vector<double, NUM_OF_JOINTS> m_vjointgoal, m_vjointdotgoal;
    Vector<double, NUM_OF_JOINTS> m_vmotorgoal, m_vmotordotgoal;



    //==============================================================================
    // CLIK Control
    //==============================================================================
    bool m_btaskmotion;
	CTrajectory PositionTrajectory, RotationTrajectory;
    int m_nCLIKerrGain;
    Vector<double, 6> m_vCLIKerr;
    Vector<double, 6> m_vX_left_heel_init, m_vX_left_toe_init;
    Matrix3d m_mR_left_heel, m_mRdes_left_heel, m_mR_left_toe, m_mRdes_left_toe;
    Vector<double, 6> m_vtaskgoal, m_vtaskdotgoal;


    //==============================================================================
    // Motion Control // 허박사님 모션으로 수정함 26.02.06/12:51
    //==============================================================================
    const std::string m_sMotionPath0 = "../utility/ref_motion/demo(260206_t2)/0-0_idle_HEAD_ROLL-1_t1.csv"; // idle
    const std::string m_sMotionPath1 = "../utility/ref_motion/demo(260206_t2)/1-1_stand_greeting2_interp_joint_theta_t2.csv";
    const std::string m_sMotionPath2 = "../utility/ref_motion/demo(260206_t2)/2-1_CHEST_POINT-2_t2.csv";
    const std::string m_sMotionPath3 = "../utility/ref_motion/demo(260206_t2)/2-2_ARMS_WELCOME-1_t2.csv";
    const std::string m_sMotionPath4 = "../utility/ref_motion/demo(260206_t2)/3-1_CHEST_POINT-2_t2.csv";
    const std::string m_sMotionPath5 = "../utility/ref_motion/demo(260206_t2)/3-2_FOREFINGER_AND_MIDDLE_FINGER_SALUTE-1_t2.csv";
    const std::string m_sMotionPath6 = "../utility/ref_motion/demo(260206_t2)/4-1_SHACKING_HAND_cut_t2.csv"; 
    const std::string m_sMotionPath7 = "../utility/ref_motion/demo(260206_t2)/5-1_HAND_V_SIGN-6_PART-1_t2.csv"; 
    const std::string m_sMotionPath8 = "../utility/ref_motion/demo(260206_t2)/5-2_THUMB_UP-2_PART-1_t2.csv";

    std::vector<Eigen::VectorXd> m_MotionData0, m_MotionData1, m_MotionData2, m_MotionData3, m_MotionData4, m_MotionData5, m_MotionData6, m_MotionData7, m_MotionData8;

    void ReadMotionCsv(std::string path, std::vector<Eigen::VectorXd>& out_data);

    Vector<double, NUM_OF_JOINTS> m_vhomeposition;

    int m_nTaskNumPad = 999;
    int m_nPreTaskNumPad = 999;
    int m_nMotionIndex = 999;      
    int m_nEndHandshake = 999;            
    int m_nMotionStep = 0;              
    int m_nNeckMotionStep = 1;              
    int m_nNeckMotioncount = 0;   
    double m_dRandomNeckWaistcount = 0;
    double m_dMotionplaybackspeed = 1.0;

    int m_nMotionIndexNum = 0;
    
    double m_dMotionStartTime = 0.0;    
    double m_dTransitionTime = 2.0;     

    Vector<double, NUM_OF_JOINTS> m_vJointQradDesPre;
    std::vector<Eigen::VectorXd>* m_pCurrentMotionData = nullptr; 

    Vector<double, NUM_OF_JOINTS> m_vqA, m_vqB;

	CTrajectory NeckTrajectory;
	CTrajectory NeckTrackingTrajectory;
    Vector<double, 2> m_vNeckgoal, m_vNeckdotgoal;
    Vector<double, 2> m_vPreNeckgoal, m_vPreNeckdotgoal;
	CTrajectory WaistTrajectory;
    Vector<double, 3> m_vWaistgoal, m_vWaistdotgoal;
    Vector<double, 3> m_vPreWaistgoal, m_vPreWaistdotgoal;

    bool m_bneckmotionfirst = true;

    double m_dneckmotiontime, m_dneckmotionendtime;
    double m_dNeckMotionStartTime = 0.0;    
    double m_nNeckFilter = 0.001;

    double m_dHandshakeStartTime = 0.0;
    double m_dHandshakeEndTime = 0.0;
    double m_nHandshakeGain = 0.0;

    int m_nNeckMotionIndex = 0;
    double m_dNeckPitchDes = 0;
    double m_dPreNeckPitchDes = 0;
    double m_dNeckYawDes = 0;
    double m_dPreNeckYawDes = 0;

    double m_dPreNeckPitchDotDes = 0;
    double m_dPreNeckYawDotDes = 0;

    double m_dNeckControlTime = 0.0;
    double m_dNeckControlStartTime = 0.0;

    Vector<double, NUM_OF_JOINTS> m_vMotionStartQ;
    Vector<double, NUM_OF_JOINTS> m_vMotionStartQdot;

    double m_dPitchCmd1 = 0.0;
    double m_dPitchCmd2 = 0.0;
    double m_dYawCmd1 = 0.0;
    double m_dYawCmd2 = 0.0;

    //==============================================================================
    // Debug and Safety
    //==============================================================================
    int m_nCoutNum, m_nCoutVelLimitNum, m_nCountTimeErr;
    Vector<double, NUM_OF_JOINTS> m_vTorqueLimitNm, m_vJointQdotradLimit, m_vMotorThetadotradLimit;
    Vector<double, NUM_OF_JOINTS> m_vJointTorqueLimitNm;
    Vector<double, NUM_OF_JOINTS> m_vJointQradUpLimit, m_vJointQradLowLimit, m_vMotorThetaradUpLimit, m_vMotorThetaradLowLimit;
    bool m_bStandStill;
    double m_dStandStillStartTime = 0.0;
    double m_dPreviousComputationTime;
    RTTIME m_rtTimeStart, m_rtTimeCheck, m_rtTimeEnd;
    bool m_bMPPItimeout;
    int m_dMPPItimeoutstep;
    long double m_ldMPPItimes;

    Vector<double, NUM_OF_JOINTS> m_vPreJointSafeDes, m_vPreRLJointSafeDes;




    //==============================================================================
    // Pre-allocated Variables for Functions
    //==============================================================================
    // For psi2q()
    Vector<double, NUM_OF_JOINTS> m_vQ_psi2q;
    double m_dTheta2_psi2q, m_dBeta_psi2q, m_dLambda_psi2q, m_dL_squared_psi2q;
    double m_dTheta1_psi2q, m_dTheta41_psi2q, m_dBeta1_psi2q, m_dLambda1_psi2q;
    double m_dTheta42_psi2q, m_dBeta2_psi2q, m_dLambda2_psi2q, m_dL__squared_psi2q;
    parallel_mechanism::ParallelMechanism WristParallelMechanism;


    // For computeActuationJacobian()
    Eigen::Matrix<double, NUM_OF_JOINTS, NUM_OF_JOINTS> m_mJ_jacobian;
    Vector<double, NUM_OF_JOINTS> m_vEpsilon_jacobian;
    Vector<double, NUM_OF_JOINTS> m_vNum_jacobian, m_vDenom_jacobian;

    // For running_cost() and terminal_cost()
    Vector<double, 14> m_vPsi;
    Vector2d m_vQ;
    double m_dIndicatorFunc;
    long double m_ldQx;
    Eigen::Matrix<double, 2, 2> m_mWq; 

    // For Compute()
    long double m_ldDenominator_compute;




    //==============================================================================
    // MPPI (Model Predictive Path Integral)
    //==============================================================================
    // MPPI Functions 
    /**
     * @brief Defines the state transition model for the MPPI controller.
     * 
     * This function predicts the next state based on the current state and control input.
     * In this implementation, it models a simple integrator where the next state is the
     * current state plus the control input scaled by the time step `delta_t`. This is used
     * by the MPPI algorithm to simulate future trajectories.
     * 
     * @param state The current state vector (motor positions).
     * @param control The control input vector (motor velocities).
     * @param delta_t The time step for the simulation.
     * @return The predicted next state vector.
     */
    Vector2d state_transition(Vector2d vState, Vector2d vControl, double dDelta_t);

    /**
     * @brief Calculates the running cost for a given state and control input in MPPI.
     * 
     * This function evaluates the cost incurred at each step of a simulated trajectory.
     * The cost is composed of:
     * - A state cost, which penalizes deviation from the desired joint position (`m_dJointQradDes`).
     * - A constraint violation cost, which heavily penalizes states that exceed joint limits (`_qlb`, `_qub`).
     * - A control cost, which penalizes large control inputs.
     * This cost is used by the MPPI algorithm to evaluate the quality of different control sequences.
     * 
     * @param state The current state vector (motor positions).
     * @param control The nominal control input vector.
     * @param controlvar The noise (perturbation) applied to the control input.
     * @return The calculated running cost as a long double.
     */
    long double running_cost(Vector2d vState, Vector2d vControl, Vector2d vControlVar);

    /**
     * @brief Calculates the terminal cost at the end of an MPPI trajectory.
     * 
     * This function evaluates the cost of the final state in a simulated trajectory.
     * The cost function is similar to the state cost in `running_cost`, penalizing
     * deviation from the desired joint position and violation of joint limits. This helps
     * guide the optimization towards ending in a desirable state.
     * 
     * @param state The terminal state vector of a trajectory.
     * @return The calculated terminal cost as a long double.
     */
    long double terminal_cost(Vector2d vState);
    
    // MPPI parameters
    int m_nState;    // Number of states
    int m_nCtrl;     // Number of control inputs
    int m_nK;          // Number of samples
    int m_nN;          // Predictive horizon
    
    double m_dDel_t;   // Time interval
    double m_dGamma;   // Discount factor
    double m_dNu;      // Exploration variance
    double m_dLambda;  // Temperature parameter

    Vector2d m_vF;     // Uncontrolled state transition vector
    Vector2d m_vQlb;   // Lower bound of joint limits (pitch, roll)
    Vector2d m_vQub;   // Upper bound of joint limits (pitch, roll)

    Matrix2d m_mR;     // Control weight matrix

    Eigen::Matrix<double, 2, 2> m_mG;          // Control matrix
    Eigen::Matrix<double, 2, 3 + 1> m_mX;     // State trajectory matrix (_n_state, _N + 1)
    Eigen::Matrix<double, 2, 3> m_mU;         // Control input sequence matrix (_n_ctrl, _N)
    Eigen::Matrix<double, 2 * 175, 3> m_mDelta_u; // Control noise samples matrix (_n_state*_K, _N)
    
    Vector<long double, 175> m_vS;      // Cost vector for each sample

    // MPPI Sampler
    std::random_device m_rd;
    std::mt19937 m_gen;
    std::normal_distribution<double> m_dist;
};

#endif
