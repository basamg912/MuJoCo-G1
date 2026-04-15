
/********************************************************************************************
* KAPEX-MuJoCo
*
* Sim2Sim Transfer to Optimize Humanoid Locomotion Strategy using Reinforcement Learning
*
*     https://github.com/S-CHOI-S/KAPEX-MuJoCo.git
*
* Advanced Robot Control Lab. (ARC)
* 	  @ Korea Institute of Science and Technology
*
*	  https://sites.google.com/view/kist-arc
*
********************************************************************************************/

/* Authors: Sol Choi (Jennifer) */

#pragma once
#ifndef __MIMIC_RL_CONTROL__
#define __MIMIC_RL_CONTROL__

#include "timer.h"

#include <ctime>
#include <cmath>
#include <atomic>
#include <memory>
#include <cstdio>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <filesystem>

#include <thread>
#include <yaml-cpp/yaml.h>

#include "string"
#include "random"

#include "StateDef.h"

// onnxruntime

#include "onnxruntime_cxx_api.h"

// IMU 헤더 추가
#include "custommath.h"
#include "../CRobot/IMU.h"

using namespace std;

#define MIMICDOF 17

class MimicRLControl
{
public:
    MimicRLControl();
    virtual ~MimicRLControl();

    /*****************************************************************************
    ** Define variables
    *****************************************************************************/
    static const size_t NUM_OBS = 82;
    static const size_t NUM_ACTIONS = 14;

    /*****************************************************************************
    ** Functions
    *****************************************************************************/
    void Initialize();
    void Control();
    void SetRobotState(const Vector<double, NUM_OF_JOINTS>& jointQrad, const Vector<double, NUM_OF_JOINTS>& jointQdotrad, const IMU_TRANSFORMED_DATA* pImuData, const Vector<double, 3>& vJoyCmd);
    const std::array<float, MIMICDOF>& GetDesiredPosition() const { return _q_des; }

    Eigen::Matrix<double, NUM_OF_JOINTS, NUM_OF_JOINTS> m_mKpRL, m_mKdRL; // RL joint control PD gain
    std::array<double, 3> m_vRLJoyCmd = {0.0, 0.0, 0.0}; // x, y, yaw
    bool m_bMimicRLfirstloop = true;      
    std::array<float, NUM_ACTIONS*2> init_ref = {
        0.000181, 0.000216, -0.000212, -0.000244, -0.047293, 0.048031, 
        0.416656, -0.418438, -0.332676, 0.333708, 0.000246, 0.000174, 
        -0.000408, 0.000481, 0.00015, 0.0001, 0.00015, 4.99997e-05, 
        -0.00470001, 0.00465009, -0.0116497, 0.0114501, -0.0231996, 0.02345, 
        0.00015, 4.99997e-05, -0.000650001, 0.000649996
    };
    std::array<float, 1> m_vinput_time_step_data = {1.0f};
    bool m_bmimic_first_loop = true;
    std::array<float, NUM_ACTIONS> RunInference();

    // Bak Debuggingtrue
    int m_nRLCoutNum = 0;
    int m_nRLCoutNum1 = 0;

private:
    /*****************************************************************************
    ** Enum Class
    *****************************************************************************/
    enum KAPEXJointIndex 
    {
      LLJ1 = 0,
      LLJ2 = 1,
      LLJ3 = 2,
      LLJ4 = 3,
      LLJ5 = 4,
      LLJ6 = 5,
      LLJ7 = 6,
      RLJ1 = 7,
      RLJ2 = 8,
      RLJ3 = 9,
      RLJ4 = 10,
      RLJ5 = 11,
      RLJ6 = 12,
      RLJ7 = 13,
      WJ1  = 14,
      WJ2  = 15,
      WJ3  = 16,
      HJ1  = 17,
      HJ2  = 18,
      LAJ1 = 19,
      LAJ2 = 20,
      LAJ3 = 21,
      LAJ4 = 22,
      LAJ5 = 23,
      LAJ6 = 24,
      LAJ7 = 25,
      RAJ1 = 26,
      RAJ2 = 27,
      RAJ3 = 28,
      RAJ4 = 29,
      RAJ5 = 30,
      RAJ6 = 31,
      RAJ7 = 32
    };

    /*****************************************************************************
    ** Variables
    *****************************************************************************/
    // YAML path
    const std::string cfg_path = "../policy/sol/config/kapex_mimic.yaml";
    
    // timestep
    float control_dt_ = 0.001;  // [1ms]: 1kHz
    size_t decimation = 20;  // [20ms]: 50Hz

    size_t step_cnt = 0; // step counter
    float motion_length = 600.f; // motion length

    // onnxruntime
    Ort::Env env_;
    Ort::AllocatorWithDefaultOptions allocator_;
    Ort::SessionOptions session_options_;

    std::unique_ptr<Ort::Session> policy_session_;

    std::vector<const char*> input_names;
    std::vector<Ort::Value> input_tensors;
    std::vector<const char*> output_names;

    std::array<float, NUM_OBS> input_data = {};
    std::array<int64_t, 2> input_shape = {1, NUM_OBS};
    std::array<int64_t, 2> time_step_shape = {1, 1}; // time_step
    Ort::Value input_tensor_{nullptr};

    /*****************************************************************************
    ** Functions
    *****************************************************************************/
    void LoadYamlConfig(const std::string& config_yaml_path);
    void PrintYamlConfig();
    void LoadOnnxModel();
    std::array<float, 3> GetGravityOrientation(const std::array<float, 4>& q);
    std::array<float, NUM_OBS> GetObservation();
    std::array<float, 6> SubtractFrameTransforms(const std::array<float, 4>& base_quat, const std::array<float, 4>& anchor_quat);

    /*****************************************************************************
    ** Parameters
    *****************************************************************************/
    // original
    // 0.f, 0.f, -0.08, 0.32, -0.24, 0.f, // 0.f,
    // 0.f, 0.f, 0.08, -0.32, 0.24, 0.f // 0.f,
    // waist3 0.2

    // com x -0.13
    // 0.0, 0.02, -0.07, +0.38, -0.33, 0.0, // 0.0,
    // 0.0, -0.02, +0.07, -0.38, +0.33, 0.0, // 0.0,
    // waist3 0.1 0.1

    // com x -0.8
    // 0.0, 0.0, -0.07, +0.38, -0.33, 0.0, 0.0,
    // 0.0, 0.0, +0.07, -0.38, +0.33, 0.0, 0.0,
    // 0.0, 0.0, 0.0

    static inline constexpr std::array<float, MIMICDOF> default_q = {
      0.f, 0.0, -0.07, 0.38, -0.33, 0.f, 0.f,
      0.f, -0.0, 0.07, -0.38, 0.33, 0.f, 0.f,
      0.f, 0.f, 0.f
      // 0.f, 0.f, 0.f,
      // 0.f, 0.f
    };

    // robot joint angle
    std::array<float, MIMICDOF> _q = {
      0.f, 0.0, -0.07, 0.38, -0.33, 0.f, 0.f,
      0.f, -0.0, 0.07, -0.38, 0.33, 0.f, 0.f,
      0.f, 0.f, 0.f
    };

    // robot init joint angle
    std::array<float, MIMICDOF> q_init = {
      0.f, 0.0, -0.07, 0.38, -0.33, 0.f, 0.f,
      0.f, -0.0, 0.07, -0.38, 0.33, 0.f, 0.f,
      0.f, 0.f, 0.f
    };

    // robot joint velocity
    std::array<float, MIMICDOF> _qdot = {};

    // robot joint force/torque
    std::array<float, MIMICDOF> _qfrc_torque = {};

    // robot base angular velocity
    std::array<float, 3> base_ang_vel = {0.f, 0.f, 0.f}; // x, y, z
    
    // robot base quaternion
    std::array<float, 4> base_quat = {1.f, 0.f, 0.f, 0.f}; // w, x, y, z

    // control
    std::array<float, MIMICDOF> _q_des = q_init; // desired joint position
    std::array<float, MIMICDOF> _qdot_des = {}; // desired joint velocity
    std::array<float, MIMICDOF> _torque = {}; // desired torque

    // control torque limit
    std::array<float, MIMICDOF> torque_limit = 
    {
      70.f, 70.f, 150.f, 150.f, 70.f, 70.f, 20.f,
      70.f, 70.f, 150.f, 150.f, 70.f, 70.f, 20.f,
      120.f, 120.f, 120.f
    };

    /////////////////////////////////////////
    // 여기 인덱싱 고쳐야 함
    /////////////////////////////////////////

    // isaaclab2mujoco
    static constexpr std::array<int, NUM_ACTIONS> isaaclab2mujoco = 
    {
      0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
    };

    // mujoco2isaaclab
    static constexpr std::array<int, NUM_ACTIONS> mujoco2isaaclab = 
    {
      LLJ1, RLJ1, LLJ2, RLJ2, LLJ3, RLJ3, LLJ4, RLJ4, LLJ5, RLJ5, LLJ6, RLJ6, LLJ7, RLJ7
    };

    // default2isaaclab
    static constexpr std::array<int, NUM_ACTIONS> default2isaaclab = 
    {
      LLJ1, RLJ1, LLJ2, RLJ2, LLJ3, RLJ3, LLJ4, RLJ4, LLJ5, RLJ5, LLJ6, RLJ6, LLJ7, RLJ7
    };

    // RL policy
    std::array<float, NUM_ACTIONS> rl_action_ = {};

    // RL idx
    static constexpr std::array<int, NUM_ACTIONS> rl2control = 
    {
      LLJ1, RLJ1, LLJ2, RLJ2, LLJ3, RLJ3, LLJ4, RLJ4,
      LLJ5, RLJ5, LLJ6, RLJ6, LLJ7, RLJ7
    };

    /*****************************************************************************
    ** Structure & Data Buffer
    *****************************************************************************/
    struct YamlConfig
    {
      std::string policy_path;
      std::string critic_path;
      std::array<float, MIMICDOF> init_kp = {};
      std::array<float, MIMICDOF> init_kd = {};
      std::array<float, MIMICDOF> rl_kp = {};
      std::array<float, MIMICDOF> rl_kd = {};
      float ang_vel_scale;
      float dof_pos_scale;
      float dof_vel_scale;
      float action_scale;
      std::array<float, 3> cmd_scale = {};
      std::array<float, 3> max_cmd = {};
    };

    struct MotionCommand
    {
      uint32_t mode;   // walk or run
      std::array<float, NUM_ACTIONS> joint_pos;
      std::array<float, NUM_ACTIONS> joint_vel;
      std::array<float, 4> achor_quat = {1, 0, 0, 0}; // w, x, y, z
    };

    // create data buffer
    YamlConfig cfg;

    // motion command
    MotionCommand ref_motion;
};

#endif //__MIMIC_RL_CONTROL__
