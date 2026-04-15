
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
#ifndef __RL_CONTROL__
#define __RL_CONTROL__

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

#define DOF 17

class RLControl
{
public:
    RLControl();
    virtual ~RLControl();

    /*****************************************************************************
    ** Define variables
    *****************************************************************************/
    static const size_t NUM_OBS = 47;
    static const size_t NUM_PRIV_OBS = 59;
    static const size_t NUM_ACTIONS = 12;

    /*****************************************************************************
    ** Functions
    *****************************************************************************/
    void Initialize();
    void Control();
    void SetRobotState(const Vector<double, NUM_OF_JOINTS>& jointQrad, const Vector<double, NUM_OF_JOINTS>& jointQdotrad, const IMU_TRANSFORMED_DATA* pImuData, const Vector<double, 3>& vJoyCmd);
    const std::array<float, DOF>& GetDesiredPosition() const { return _q_des; }

    Eigen::Matrix<double, NUM_OF_JOINTS, NUM_OF_JOINTS> m_mKpRL, m_mKdRL; //RL joint control PD gain
    std::array<double, 3> m_vRLJoyCmd = {0.0, 0.0, 0.0}; // x, y, yaw
    std::array<float, 3> m_vGravity_orientation = {0.f, 0.f, 0.f}; // w, x, y, z

    //Bak Debugging
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
    const std::string cfg_path = "../policy/sol/config/kapex.yaml";
    
    // timestep
    float control_dt_ = 0.001;  // [1ms]: 1kHz
    size_t decimation = 20;  // [20ms]: 50Hz

    size_t step_cnt = 0; // step counter

    // onnxruntime
    Ort::Env env_;
    Ort::AllocatorWithDefaultOptions allocator_;
    Ort::SessionOptions session_options_;

    std::unique_ptr<Ort::Session> policy_session_;
    std::unique_ptr<Ort::Session> critic_session_;

    std::vector<const char*> input_names;
    std::vector<Ort::Value> input_tensors;
    std::vector<const char*> output_names;
    std::vector<const char*> privileged_input_names;
    std::vector<Ort::Value> privileged_input_tensors;
    std::vector<const char*> privileged_output_names;

    std::array<float, NUM_OBS> input_data = {};
    std::array<int64_t, 2> input_shape = {1, NUM_OBS};
    Ort::Value input_tensor_{nullptr};
    std::array<float, NUM_PRIV_OBS> privileged_input_data = {};
    std::array<int64_t, 2> privileged_input_shape = {1, NUM_PRIV_OBS};
    Ort::Value privileged_input_tensor_{nullptr};

    /*****************************************************************************
    ** Functions
    *****************************************************************************/
    void LoadYamlConfig(const std::string& config_yaml_path);
    void PrintYamlConfig();
    void LoadOnnxModel();
    std::array<float, 3> GetGravityOrientation(const std::array<float, 4>& q);
    std::array<float, NUM_OBS> GetObservation();
    std::array<float, NUM_PRIV_OBS> GetPrivilegedObservation();
    std::array<float, NUM_ACTIONS> RunInference();
    float RunPrivilegedInference();

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

    // RL 기본자세 바꾸는 곳(@@@@@)
    static inline constexpr std::array<float, DOF> default_q = {
      // 0.f, 0.f, -0.08, 0.32, -0.24, 0.f, 0.f,
      // 0.f, 0.f, 0.08, -0.32, 0.24, 0.f, 0.f,
      // 0.f, 0.f, 0.2f

      0.03, 0.06, -0.07, +0.38, -0.33, 0.0, 0.0,
      -0.03, -0.06, +0.07, -0.38, +0.33, 0.0, 0.0,
      0.0, 0.0, 0.0

      // 0.f, 0.f,
      // 0.2f, 0.1f, 0.f, -0.3f, 0.f, 0.f, 0.f,
      // -0.2f, -0.1f, 0.f, 0.3f, 0.f, 0.f, 0.f
    };

    // robot joint angle
    std::array<float, DOF> _q = {
      0.03, 0.06, -0.07, +0.38, -0.33, 0.0, 0.0,
      -0.03, -0.06, +0.07, -0.38, +0.33, 0.0, 0.0,
      0.0, 0.0, 0.0
    };

    // robot init joint angle
    std::array<float, DOF> q_init = {
      // 0.f, 0.f, 0.859, 1.f, 0.f, 0.f, 0.f, // floating base
      // 0.f, 0.f, -0.08, 0.32, -0.24, 0.f, 0.f,
      // 0.f, 0.f, 0.08, -0.32, 0.24, 0.f, 0.f,
      // 0.f, 0.f, 0.2fs

      0.03, 0.06, -0.07, +0.38, -0.33, 0.0, 0.0,
      -0.03, -0.06, +0.07, -0.38, +0.33, 0.0, 0.0,
      0.0, 0.0, 0.0

      // 0.f, 0.f,
      // 0.2, 0.1, 0.f, -0.3, 0.f, 0.f, 0.f,
      // -0.2, -0.1, 0.f, 0.3, 0.f, 0.f, 0.f
    };

    // robot joint velocity
    std::array<float, DOF> _qdot = {};

    // robot joint force/torque
    std::array<float, DOF> _qfrc_torque = {};

    // robot base angular velocity
    std::array<float, 3> base_ang_vel = {0.f, 0.f, 0.f}; // x, y, z
    
    // robot base quaternion
    std::array<float, 4> base_quat = {1.f, 0.f, 0.f, 0.f}; // w, x, y, z

    // control
    std::array<float, DOF> _q_des = q_init; // desired joint position
    std::array<float, DOF> _qdot_des = {}; // desired joint velocity
    std::array<float, DOF> _torque = {}; // desired torque

    // control torque limit
    std::array<float, DOF> torque_limit = 
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
      0, 2, 4, 6, 8, 10, 1, 3, 5, 7, 9, 11
      // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
    };

    // mujoco2isaaclab
    static constexpr std::array<int, NUM_ACTIONS> mujoco2isaaclab = 
    {
      LLJ1, RLJ1, LLJ2, RLJ2, LLJ3, RLJ3, LLJ4, RLJ4, LLJ5, RLJ5, LLJ6, RLJ6
    };

    // default2isaaclab
    static constexpr std::array<int, NUM_ACTIONS> default2isaaclab = 
    {
      LLJ1, RLJ1, LLJ2, RLJ2, LLJ3, RLJ3, LLJ4, RLJ4, LLJ5, RLJ5, LLJ6, RLJ6
    };

    // RL policy
    float stride_a = 8.0e-7;
    float stride_b = 1.0;
    float eps = 1e-07;
    std::array<float, NUM_ACTIONS> rl_action_ = {};
    float rl_value_ = 0.f;

    // RL idx
    static constexpr std::array<int, NUM_ACTIONS> rl2control = 
    {
      LLJ1, LLJ2, LLJ3, LLJ4, LLJ5, LLJ6,
      RLJ1, RLJ2, RLJ3, RLJ4, RLJ5, RLJ6
    };

    /*****************************************************************************
    ** Structure & Data Buffer
    *****************************************************************************/
    struct YamlConfig
    {
      std::string policy_path;
      std::string critic_path;
      std::array<float, DOF> init_kp = {};
      std::array<float, DOF> init_kd = {};
      std::array<float, DOF> rl_kp = {};
      std::array<float, DOF> rl_kd = {};
      float ang_vel_scale;
      float dof_pos_scale;
      float dof_vel_scale;
      float action_scale;
      std::array<float, 3> cmd_scale = {};
      std::array<float, 3> max_cmd = {};
    };

    struct VelocityCommand
    {
      uint32_t mode;   // walk or run
      std::array<float, 3> lin_vel;
      std::array<float, 3> ang_vel;
    };

    // create data buffer
    YamlConfig cfg;

    // velocity command
    VelocityCommand cmd_vel;
};

#endif //__RL_CONTROL__
