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

#include "MimicRLControl.h"
#include "custommath.h"

MimicRLControl::MimicRLControl()
{
	Initialize();
}


MimicRLControl::~MimicRLControl()
{

}


void MimicRLControl::Initialize()
{
	LoadYamlConfig(cfg_path);
  LoadOnnxModel();

	std::cout << "RL Control Initialized" << std::endl;
}


void MimicRLControl::LoadYamlConfig(const std::string& config_yaml_path)
{
  YAML::Node config = YAML::LoadFile(config_yaml_path);

  const std::string var_name = "{KAPEX_POLICY_CFG_DIR}";
  std::string replacement = "../policy/sol/config";

  std::string policy_path = config["policy_path"].as<std::string>();

  size_t pos = policy_path.find(var_name);
  if (pos != std::string::npos)
  {
  	policy_path.replace(pos, var_name.length(), replacement);
  }

  // onnx path
  cfg.policy_path = policy_path;

  // init kp
  std::vector<float> init_kp = config["init_kp"].as<std::vector<float>>();
  std::copy(init_kp.begin(), init_kp.end(), cfg.init_kp.begin());

  // init kd
  std::vector<float> init_kd = config["init_kd"].as<std::vector<float>>();
  std::copy(init_kd.begin(), init_kd.end(), cfg.init_kd.begin());

  // RL kp
  std::vector<float> rl_kp = config["rl_kp"].as<std::vector<float>>();
  std::copy(rl_kp.begin(), rl_kp.end(), cfg.rl_kp.begin());

  // RL kd
  std::vector<float> rl_kd = config["rl_kd"].as<std::vector<float>>();
  std::copy(rl_kd.begin(), rl_kd.end(), cfg.rl_kd.begin());

  m_mKpRL.setZero();
  m_mKdRL.setZero();

  for (int i = 0; i < 17; i++)
  {
    m_mKpRL(i, i) = cfg.rl_kp[i];
    m_mKdRL(i, i) = cfg.rl_kd[i];
  }

  // scale factors
  cfg.ang_vel_scale = config["ang_vel_scale"].as<float>();
  cfg.dof_pos_scale = config["dof_pos_scale"].as<float>();
  cfg.dof_vel_scale = config["dof_vel_scale"].as<float>();
  cfg.action_scale = config["action_scale"].as<float>();

  PrintYamlConfig();
}


void MimicRLControl::PrintYamlConfig()
{
  std::cout << "\n============================== [MimicRL Config Loaded] ==============================\n";
  std::cout << "  Model path       : " << cfg.policy_path << "\n";

  // std::cout << "  Init Kp size     : " << cfg.init_kp.size() << "\n";
  // std::cout << "  Init Kd size     : " << cfg.init_kd.size() << "\n";
  // std::cout << "  RL Kp size       : " << cfg.rl_kp.size() << "\n";
  // std::cout << "  RL Kd size       : " << cfg.rl_kd.size() << "\n";

  std::cout << "  Scale factors:\n";
  std::cout << "    ang_vel_scale  : " << cfg.ang_vel_scale << "\n";
  std::cout << "    dof_pos_scale  : " << cfg.dof_pos_scale << "\n";
  std::cout << "    dof_vel_scale  : " << cfg.dof_vel_scale << "\n";
  std::cout << "    action_scale   : " << cfg.action_scale << "\n";
  std::cout << "================================================================================\n" << std::endl;
}


void MimicRLControl::LoadOnnxModel()
{
  // initialize onnx runtime env
  env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "KAPEX_MIMIC");

  // session options
  Ort::SessionOptions session_options;
  session_options.SetIntraOpNumThreads(1);
  session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

  // load onnx model
  policy_session_ = std::make_unique<Ort::Session>(env_, cfg.policy_path.c_str(), session_options);

  // Prepare input/output names
  const char* input_name0 = policy_session_->GetInputName(0, allocator_);
  const char* input_name1 = policy_session_->GetInputName(1, allocator_);

  const char* output_name0 = policy_session_->GetOutputName(0, allocator_);
  const char* output_name1 = policy_session_->GetOutputName(1, allocator_);
  const char* output_name2 = policy_session_->GetOutputName(2, allocator_);
  const char* output_name3 = policy_session_->GetOutputName(3, allocator_);
  const char* output_name4 = policy_session_->GetOutputName(4, allocator_);
  const char* output_name5 = policy_session_->GetOutputName(5, allocator_);
  const char* output_name6 = policy_session_->GetOutputName(6, allocator_);

  // input/output names
  input_names = {input_name0, input_name1};
  output_names = {output_name0, output_name1, output_name2,
                  output_name3, output_name4, output_name5, output_name6};
}

std::array<float, 3> MimicRLControl::GetGravityOrientation(const std::array<float, 4>& q)
{
  float qw = q[0];
  float qx = q[1];
  float qy = q[2];
  float qz = q[3];

  std::array<float, 3> gravity_orientation;

  gravity_orientation[0] = 2.0f * (-qz * qx + qw * qy);
  gravity_orientation[1] = -2.0f * (qz * qy + qw * qx);
  gravity_orientation[2] = 1.0f - 2.0f * (qw * qw + qz * qz);

  return gravity_orientation;
}

std::array<float, 6> MimicRLControl::SubtractFrameTransforms(const std::array<float, 4>& base_quat, const std::array<float, 4>& anchor_quat)
{
  std::array<float, 6> result{0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

  constexpr float kEps = 1e-9f;

  const float w1 = base_quat[0], x1 = base_quat[1], y1 = base_quat[2], z1 = base_quat[3];
  const float n2 = w1*w1 + x1*x1 + y1*y1 + z1*z1;
  const float denom = (n2 < kEps) ? kEps : n2;
  const float iw =  w1 / denom;
  const float ix = -x1 / denom;
  const float iy = -y1 / denom;
  const float iz = -z1 / denom;

  const float w2 = anchor_quat[0], x2 = anchor_quat[1], y2 = anchor_quat[2], z2 = anchor_quat[3];
  const float qw = iw*w2 - ix*x2 - iy*y2 - iz*z2;
  const float qx = iw*x2 + ix*w2 + iy*z2 - iz*y2;
  const float qy = iw*y2 - ix*z2 + iy*w2 + iz*x2;
  const float qz = iw*z2 + ix*y2 - iy*x2 + iz*w2;

  const float qn2   = qw*qw + qx*qx + qy*qy + qz*qz;
  const float two_s = 2.0f / ((qn2 < kEps) ? kEps : qn2);

  const float r = qw, i = qx, j = qy, k = qz;

  const float m00 = 1.f - two_s * (j*j + k*k);
  const float m01 =       two_s * (i*j - k*r);
  const float m02 =       two_s * (i*k + j*r);

  const float m10 =       two_s * (i*j + k*r);
  const float m11 = 1.f - two_s * (i*i + k*k);
  const float m12 =       two_s * (j*k - i*r);

  const float m20 =       two_s * (i*k - j*r);
  const float m21 =       two_s * (j*k + i*r);
  const float m22 = 1.f - two_s * (i*i + j*j);

  result[0] = m00;
  result[1] = m01;
  result[2] = m10;
  result[3] = m11;
  result[4] = m20;
  result[5] = m21;

  return result;
}

std::array<float, MimicRLControl::NUM_OBS> MimicRLControl::GetObservation()
{
  size_t start_idx = 2 * NUM_ACTIONS + 12; // start index for joint pos
  std::array<float, NUM_OBS> obs = {};
  std::array<float, 6> ref_anchor_orientation = SubtractFrameTransforms(base_quat, ref_motion.achor_quat);
  std::array<float, 3> gravity_orientation = GetGravityOrientation(base_quat);

  // reference motion (command)
  for (int i = 0; i < NUM_ACTIONS; ++i)
  {
    int m2i_idx = mujoco2isaaclab[i];
    obs[i] = ref_motion.joint_pos[i];
  }

  for (int i = 0; i < NUM_ACTIONS; ++i)
  {
    obs[i + NUM_ACTIONS] = ref_motion.joint_vel[i];
  }
  
  // reference anchor orientation
  for (int i = 0; i < 6; ++i)
  {
    obs[i + 2 * NUM_ACTIONS] = ref_anchor_orientation[i];
  }

  // ang_vel
  for (int i = 0; i < 3; ++i)
  {
    obs[i + 2 * NUM_ACTIONS + 6] = base_ang_vel[i] * cfg.ang_vel_scale;
  }

  // projected gravity
  for (int i = 0; i < 3; ++i)
  {
    obs[i + 2 * NUM_ACTIONS + 6 + 3] = gravity_orientation[i];
  }

  // joint pos
  for (size_t i = 0; i < NUM_ACTIONS; ++i)
  {
    int m2i_idx = mujoco2isaaclab[i];
    int d2i_idx = default2isaaclab[i];
    obs[start_idx + i] = (_q[m2i_idx] - default_q[d2i_idx]) * cfg.dof_pos_scale;
  }

  // joint vel
  start_idx += NUM_ACTIONS;
  for (size_t i = 0; i < NUM_ACTIONS; ++i)
  {
    int m2i_idx = mujoco2isaaclab[i];
    obs[start_idx + i] = _qdot[m2i_idx] * cfg.dof_vel_scale;
  }

  // action
  std::copy(rl_action_.begin(), rl_action_.end(), obs.begin() + start_idx + NUM_ACTIONS);

  return obs;
}

std::array<float, MimicRLControl::NUM_ACTIONS> MimicRLControl::RunInference()
{
  // Get observation data
  input_data = GetObservation();

  if(m_bMimicRLfirstloop){
    for(int i=0; i<28; i++){  
      input_data[i] = init_ref[i];
    } 
    m_bMimicRLfirstloop = false;
  }
  
  // for test
  // input_data = {7.53429e-05, -0.0979283, -6.02641e-05, -0.0109637, -1.323e-05, -0.99994, 0, 0, 0, -3.96689e-06, 4.90339e-06, 0.000145784, -0.000158202, 0.00176772, -0.00177035, 0.0121048, -0.0121375, -0.00392088, 0.00396207, 0.000105976, -0.000110627, -1.79908e-05, 2.69351e-05, 3.67926e-05, -0.000162455, -0.0720198, 0.0726888, 0.435609, -0.436781, -0.262937, 0.263583, 0.000272864, -0.000199852, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  if(m_nRLCoutNum <= 5){
    cout<<"input_data"<<endl;
    for(int i=0; i<28; i++){ //cmd
      cout<<input_data[i]<<" ";
    }
    cout<<endl;
    for(int i=28; i<34; i++){ //anc
      cout<<input_data[i]<<" ";
    }
    cout<<endl;
    for(int i=34; i<40; i++){ // base 어쩌고
      cout<<input_data[i]<<" ";
    }
    cout<<endl;
    for(int i=40; i<68; i++){ // joint
      cout<<input_data[i]<<" ";
    }
    cout<<endl;
    for(int i=68; i<82; i++){ // act
      cout<<input_data[i]<<" ";
    }
    cout<<endl;
    cout<<"quaternion"<<endl;
    for(int i = 0; i<base_quat.size(); i++){
      cout<<base_quat[i]<<" ";
    }
    cout<<endl;
    m_nRLCoutNum = 0;
  }
  m_nRLCoutNum ++;

  // create the input tensor with observation data
  Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_data.data(), input_data.size(), input_shape.data(), input_shape.size());

  // create tensors for input_time_step
  Ort::Value input_time_step_tensor = Ort::Value::CreateTensor<float>(memory_info, m_vinput_time_step_data.data(), m_vinput_time_step_data.size(), time_step_shape.data(), time_step_shape.size());

  // create input tensors (input_tensor, input_time_step_tensor)
  input_tensors.clear();
  input_tensors.push_back(std::move(input_tensor));
  input_tensors.push_back(std::move(input_time_step_tensor));

  // run the inference
  std::vector<Ort::Value> output_tensors;
  try 
  {
    output_tensors = policy_session_->Run(Ort::RunOptions{nullptr}, input_names.data(), input_tensors.data(), 2, output_names.data(), 7);
  } 
  catch (const std::exception& e) 
  {
    std::cerr << "Inference failed: " << e.what() << std::endl;
  }

  // process the output tensor
  float* output_data = output_tensors[0].GetTensorMutableData<float>();
  size_t output_size = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();

  float* output_joint_pos_data = output_tensors[1].GetTensorMutableData<float>();
  size_t output_joint_pos_size = output_tensors[1].GetTensorTypeAndShapeInfo().GetElementCount();

  float* output_joint_vel_data = output_tensors[2].GetTensorMutableData<float>();
  size_t output_joint_vel_size = output_tensors[2].GetTensorTypeAndShapeInfo().GetElementCount();

  float* output_body_pos_w_data = output_tensors[3].GetTensorMutableData<float>();
  size_t output_body_pos_w_size = output_tensors[3].GetTensorTypeAndShapeInfo().GetElementCount();

  float* output_body_quat_w_data = output_tensors[4].GetTensorMutableData<float>();
  size_t output_body_quat_w_size = output_tensors[4].GetTensorTypeAndShapeInfo().GetElementCount();

  float* output_body_lin_vel_w_data = output_tensors[5].GetTensorMutableData<float>();
  size_t output_body_lin_vel_w_size = output_tensors[5].GetTensorTypeAndShapeInfo().GetElementCount();

  float* output_body_ang_vel_w_data = output_tensors[6].GetTensorMutableData<float>();
  size_t output_body_ang_vel_w_size = output_tensors[6].GetTensorTypeAndShapeInfo().GetElementCount();

  // update the command joint_pos & joint_vel
  std::copy(output_joint_pos_data, output_joint_pos_data + output_joint_pos_size, ref_motion.joint_pos.begin());
  std::copy(output_joint_vel_data, output_joint_vel_data + output_joint_vel_size, ref_motion.joint_vel.begin());
  std::copy(output_body_quat_w_data, output_body_quat_w_data + 4, ref_motion.achor_quat.begin());  

  if (output_size != NUM_ACTIONS) {
    std::cerr << "\033[31m[ERROR]\033[0m Policy/Output size mismatch: got " << output_size
              << ", expected " << NUM_ACTIONS << std::endl;
    return std::array<float, NUM_ACTIONS>{};
  }

  std::array<float, NUM_ACTIONS> output_action = {};
  std::copy(output_data, output_data + output_size, output_action.begin());

  return output_action;
}

void MimicRLControl::Control() // 1kHz
{
	// // PD Control
  //   for (size_t i = 0; i < MIMICDOF; ++i)
  //   {
  //     _torque[i] = cfg.rl_kp[i] * (_q_des[i] - _q[i]) - cfg.rl_kd[i] * _qdot[i];

  //     // clamp torque limit
  //     if (abs(_torque[i]) > torque_limit[i])
  //       _torque[i] = std::clamp(_torque[i], -torque_limit[i], torque_limit[i]);
  //   }

  //   step_cnt++;

  //   if (step_cnt % decimation == 0) // 50Hz
  //   {
  //     // Run inference
  //     rl_action_ = RunInference();
  //     rl_value_ = RunPrivilegedInference();

  //     // cout << "\033[32m[INFO]\033[0m RL Value: " << rl_value_ << endl;

  //     // Update the desired joint positions with the rl action
  //     for (size_t i = 0; i < NUM_ACTIONS; ++i)
  //     {
  //       int r2c_idx = rl2control[i];
  //       int i2m_idx = isaaclab2mujoco[i];
  //       _q_des[r2c_idx] = (rl_action_[i2m_idx] * cfg.action_scale) + default_q[i];
  //     }
  //   }

  ///////////////////////////////////////////////////////////////////////////////////
	// 위 기본 코드 바탕 real 코드 (by Bak)
  step_cnt += decimation;

  // Run inference
  rl_action_ = RunInference();
  
  if (m_vinput_time_step_data[0] > motion_length - 1.0f)
  {
    m_vinput_time_step_data[0] = 1.0f;
  }
  else
  {
    m_vinput_time_step_data[0] += 1.0f;
  }

  // if(m_nRLCoutNum < 2){
  //   m_nRLCoutNum ++;
  //   for(int i=0; i<NUM_ACTIONS; i++){
  //     rl_action_[i] = 0.0;
  //   }
  // }

  // cout << "\033[32m[INFO]\033[0m RL Action: " << rl_action_ << endl;
  // cout << "\033[32m[INFO]\033[0m RL Value: " << rl_value_ << endl;

  // Update the desired joint positions with the rl action
  for (size_t i = 0; i < NUM_ACTIONS; ++i)
  {
    int r2c_idx = rl2control[i];
    int i2m_idx = isaaclab2mujoco[i];
    _q_des[r2c_idx] = (rl_action_[i2m_idx] * cfg.action_scale) + default_q[r2c_idx];
  }
}


void MimicRLControl::SetRobotState(const Vector<double, NUM_OF_JOINTS>& jointQrad, const Vector<double, NUM_OF_JOINTS>& jointQdotrad, const IMU_TRANSFORMED_DATA* pImuData, const Vector<double, 3>& vJoyCmd)
{
    
    for (size_t i = 0;  i < MIMICDOF; ++i) {
        _q[i] = static_cast<float>(jointQrad(i));
    }
    
    
    for (size_t i = 0; i < MIMICDOF; ++i) {
        _qdot[i] = static_cast<float>(jointQdotrad(i));
    }
    
    
    if (pImuData != nullptr) {
        
        base_quat[0] = static_cast<float>(pImuData->quat(0)); 
        base_quat[1] = static_cast<float>(pImuData->quat(1)); 
        base_quat[2] = static_cast<float>(pImuData->quat(2));
        base_quat[3] = static_cast<float>(pImuData->quat(3));
        
        
        base_ang_vel[0] = static_cast<float>(pImuData->omega(0)); 
        base_ang_vel[1] = static_cast<float>(pImuData->omega(1)); 
        base_ang_vel[2] = static_cast<float>(pImuData->omega(2)); 
    }

    m_vRLJoyCmd[0] = static_cast<double>(vJoyCmd(0));
    m_vRLJoyCmd[1] = static_cast<double>(vJoyCmd(1));
    m_vRLJoyCmd[2] = static_cast<double>(vJoyCmd(2));

    // cout<<"imu rl base quat"<<endl;
    // for(int i = 0; i<4; i++){
    //   cout<<base_quat[i]<<" ";
    // }
    // cout<<endl;
    // cout<<"imu rl base ang vel"<<endl;
    // for(int i = 0; i<3; i++){
    //   cout<<base_ang_vel[i]<<" ";
    // }
    // cout<<endl;

    // if(m_nRLCoutNum1 == 5){
    //   cout<<"_q"<<endl;
    //   for(int i=0; i<MIMICDOF; i++){
    //     cout<<_q[i]<<" ";
    //   }
    //   cout<<endl;
    //   cout<<"_qdot"<<endl;
    //   for(int i=0; i<MIMICDOF; i++){
    //     cout<<_qdot[i]<<" ";
    //   }
    //   cout<<endl;
    //   m_nRLCoutNum1 = 0;
    // }
    // m_nRLCoutNum1 ++;

    //for testing
    // _q = {
    //   0.f, 0.f, -0.08, 0.32, -0.24, 0.f,
    //   0.f, 0.f, 0.08, -0.32, 0.24, 0.f
    // };
    // _qdot = {
    //   0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    //   0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    // };
    // base_quat = {1.0f, 0.0f, 0.0f, 0.0f};
    // base_ang_vel = {0.0f, 0.0f, 0.0f};
}


/*
input_data: 
7.53429e-05, -0.0979283, -6.02641e-05, -0.0109637, -1.323e-05, -0.99994, 0, 0, 0, -3.96689e-06, 4.90339e-06, 0.000145784, -0.000158202, 0.00176772, -0.00177035, 0.0121048, -0.0121375, -0.00392088, 0.00396207, 0.000105976, -0.000110627, -1.79908e-05, 2.69351e-05, 3.67926e-05, -0.000162455, -0.0720198, 0.0726888, 0.435609, -0.436781, -0.262937, 0.263583, 0.000272864, -0.000199852, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
rl_action: 
-0.14926, 0.241496, 0.110616, -0.120663, -0.163648, -0.335109, -0.193312, 0.557254, 0.349424, -0.195311, -0.235131, 0.0650167, 
real_action:
-0.14926 0.241496 0.110616 -0.120663 -0.163648 -0.335109 -0.193312 0.557254 0.349424 -0.195311 -0.235131 0.0650167 
input_data: 
-0.267875, -0.245583, 0.184084, -0.0150138, 0.00326238, -0.999882, 0, 0, 0, -0.000946149, -0.000813499, 0.00432472, 0.00413644, 0.00707062, -0.0186933, 0.0129185, 0.0101933, -0.00556664, -0.00515571, 9.91945e-05, -0.00014506, -0.0873822, -0.0772029, 0.336407, 0.346104, 0.488122, -1.45241, -0.275645, 2.21072, 0.061247, -0.980915, -0.00111352, -0.00291189, -0.14926, 0.241496, 0.110616, -0.120663, -0.163648, -0.335109, -0.193312, 0.557254, 0.349424, -0.195311, -0.235131, 0.0650167, 0, 0, 
rl_action: 
-0.107307, 0.24068, -0.0289138, -0.242057, -0.191084, -0.281491, -0.230496, 1.11035, 0.535773, -0.321095, -0.322798, 0.096757, 
input_data: 
-0.368718, -0.131036, 0.0738344, -0.0190301, 0.0107808, -0.999761, 0, 0, 0, -0.00553226, -0.00568738, 0.0113789, 0.0109589, 0.0202363, -0.0639211, -0.000691235, 0.0906418, -0.000133529, -0.043962, 0.000574353, 0.0014448, -0.276856, -0.303834, 0.292732, 0.273895, 0.714017, -2.73376, -0.94536, 5.22139, 0.426567, -2.60011, 0.020198, 0.112598, -0.107307, 0.24068, -0.0289138, -0.242057, -0.191084, -0.281491, -0.230496, 1.11035, 0.535773, -0.321095, -0.322798, 0.096757, 0, 0, 
rl_action: 
0.0111749, 0.258949, -0.0981946, -0.274832, -0.174056, -0.430377, -0.183452, 1.55683, 0.51151, -0.386289, -0.2452, 0.191442,
*/