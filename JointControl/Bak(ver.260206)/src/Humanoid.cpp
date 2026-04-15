
#include "Humanoid.h"
#include <unistd.h>
#include <fstream>
#include <random>
#include <map>
#include "FilePrint.h"
#include "Hand.h"

using namespace rclcpp;

void proc_main_control(void* apRobot);
void proc_ethercat_control(void* apRobot);
void proc_keyboard_control(void* apRobot);
void proc_terminal_output(void* apRobot);
void proc_logger(void* apRobot);
void proc_learning(void* apRobot);
void proc_imu(void *);
void proc_gamepad_control(void* apRobot);



CHumanoid::CHumanoid(CConfigRobot* apConfig)
{
    m_cKeyPress = ' ';
    // m_pcEcatEpos4 = NULL;
    m_bStopTask = FALSE;
    m_strDataLog = "DataLog_";
    m_nEcatCycle = 0;
    m_pcEcatElmo = NULL;

    m_bEcatOP = FALSE;

    m_AxisCmd.target_task = 0;
    m_AxisCmd.axis_id.assign(33, 0);
    m_AxisCmd.control_modes.assign(33, 0);
    m_AxisCmd.target_positions.assign(33, 0.0);
    m_AxisCmd.target_postion_hands.assign(20, 0.0);
    
    m_stAxisState.axis_id.assign(33,0);
    m_stAxisState.control_modes.assign(33, 0);
    m_stAxisState.enabled.assign(33, FALSE);
    m_stAxisState.positions.assign(33, 0.0);
    m_stAxisState.positions_hands.assign(20, 0.0);
    m_stAxisState.task = 0;
    m_stAxisState.velocities.assign(33, 0.0);
    m_stAxisState.torques.assign(33, 0.0);
    

    m_pcJointControl = new CJointControl();
    m_pcRLControl = new RLControl();
    m_pcMimicRLControl = new MimicRLControl();
    m_pcGamepad = new CGamepad();
    
    
    /* Initialize Configuration */
    if (apConfig == NULL)
    {
        m_pcConfigRobot = new CConfigRobot();
        m_pcConfigRobot->LoadDefaultConfig();
    }
    else
    {
        m_pcConfigRobot = apConfig;
        m_pcConfigRobot->ReadConfiguration();
    }
    /* Set Robot Name */
    m_strName = m_pcConfigRobot->GetSystemConf().strName;

    // 
    
    m_eControlState = IDLE;
    
    
    /* todo: 1. check return values, 2. include fucnction pointers in the loop without using void vectors */
    /* this is temporary */
    AddTaskFunction(&proc_main_control);    
    AddTaskFunction(&proc_ethercat_control);    
    AddTaskFunction(&proc_keyboard_control);
    AddTaskFunction(&proc_terminal_output);
    AddTaskFunction(&proc_logger);
    AddTaskFunction(&proc_imu);    
    AddTaskFunction(&proc_learning);
    AddTaskFunction(&proc_gamepad_control);
    
    

    m_pcRos2Executor = new CROS2Executor[4];

    m_pcPublisher_AxisInfoState = new CROS2AxisInfoStatePub("AxisInfoStatePub", "/nuc/axis_info/states");
    
    m_pcRos2Executor[0].AddNode(m_pcPublisher_AxisInfoState);
    m_pcRos2Executor[0].Init();
    m_pcPublisher_AxisInfoState->SetPeriod(10);

    m_pcSubscriber_AxisInfoCmd = new CROS2AxisInfoCmdSub("AxisInfoCmdSub", "/orin/axis_info/cmds");
    m_pcSubscriber_AxisInfoCmd->RegisterCallbackSubscriber(std::bind(&CHumanoid::OnRecvROS2AxisInfoCmd, 
                                                        this, 
                                                        std::placeholders::_1, 
                                                        std::placeholders::_2, 
                                                        std::placeholders::_3, 
                                                        std::placeholders::_4));
    
    m_pcRos2Executor[1].AddNode(m_pcSubscriber_AxisInfoCmd);
    m_pcRos2Executor[1].Init();


    // Initialize hand target position (Hyesung)
    m_vecRightHandCurrentRefPos.assign(20, 0.0);
    m_vecLeftHandCurrentRefPos.assign(20, 0.0);

    m_vecRightHandStartPos.assign(20, 0.0);
    m_vecLeftHandStartPos.assign(20, 0.0);
    
    m_vecRightHandEndPos.assign(20, 0.0);
    m_vecLeftHandEndPos.assign(20, 0.0);

    // Read csv motion data (Hyesung)
    // std::string file_path = "/home/moon/Hand/Motion/SHACKING_HAND.csv";
    // LoadMotionDataCSV(file_path);

    // m_pcRos2ExecutorAdditional = new CROS2Executor(20);

    // m_pcSubscriber_Imu = new CROS2ImuSub("ImuSub", "/imu/data");
    // m_pcSubscriber_Imu->RegisterCallbackSubscriber(std::bind(&CHumanoid::OnRecvROS2Imu, 
    //                                                     this, 
    //                                                     std::placeholders::_1, 
    //                                                     std::placeholders::_2, 
    //                                                     std::placeholders::_3, 
    //                                                     std::placeholders::_4));
    
    // m_pcRos2Executor[0].AddNode(m_pcSubscriber_Imu);
    // m_pcRos2Executor[0].Init();

    // m_pcSubscriber_PointCloud = new CROS2PointCloudSub("PointCloudSub", "/ouster/points");
    // m_pcSubscriber_PointCloud->RegisterCallbackSubscriber(std::bind(&CHumanoid::OnRecvROS2PointCloud, 
    //                                                     this, 
    //                                                     std::placeholders::_1, 
    //                                                     std::placeholders::_2, 
    //                                                     std::placeholders::_3, 
    //                                                     std::placeholders::_4));

    // m_pcPublisher_PointCloud = new CROS2PointCloudPub("PointCloud", "/ouster/points_compressed");

    // m_pcRos2Executor[0].AddNode(m_pcSubscriber_PointCloud);
    // m_pcRos2Executor[0].Init();
    
    
    // // m_pcPublisher_PointCloud->SetPeriod(100);    
    // // m_pcRos2Executor[2].AddNode(m_pcPublisher_PointCloud);
    // // m_pcRos2Executor[2].Init();
    
    // // m_pcRos2Executor[1] = CROS2Executor();
    

    // // m_pcRos2ExecutorAdditional->AddNode(m_pcSubscriber_PointCloud);
    // // m_pcRos2ExecutorAdditional->Init();

    

    // m_pcSubscriber_AxisInfoCmd = new CROS2AxisInfoCmdSub("AxisInfoCmdSub", "/axis_info/compressed_commands");
    // m_pcSubscriber_AxisInfoCmd->RegisterCallbackSubscriber(std::bind(&CHumanoid::OnRecvROS2AxisInfoCmd, 
    //                                                     this, 
    //                                                     std::placeholders::_1, 
    //                                                     std::placeholders::_2, 
    //                                                     std::placeholders::_3, 
    //                                                     std::placeholders::_4));
    
    // m_pcRos2Executor[2].AddNode(m_pcSubscriber_AxisInfoCmd);
    // m_pcRos2Executor[2].Init();
}

CHumanoid::~CHumanoid()
{
    // if (NULL != m_pcConfigRobot)
    // {
    //     delete m_pcConfigRobot;
    //     m_pcConfigRobot = NULL;
    // }

    

    // m_cExecutor.DeInit();
    
    
    
}

BOOL 
CHumanoid::Init(BOOL abSim)
{
    BOOL flag = CRobot::Init(abSim);
    
    SetJointPDGain();

    return flag;
}

BOOL 
CHumanoid::DeInit()
{
    m_pcRos2Executor[0].DeInit();
    m_pcRos2Executor[1].DeInit();
    m_pcRos2Executor[2].DeInit();
    m_pcRos2Executor[3].DeInit();


    

    if (TRUE == CRobot::DeInit())
    {
        
        

        if (NULL != m_pcEcatElmo)
        {
            delete[] m_pcEcatElmo;
            m_pcEcatElmo = NULL;            
        }

        if (NULL != m_pcTerminals)
        {
            delete[] m_pcTerminals;
            m_pcTerminals = NULL;
        }


        if (NULL != m_pcHand) {
            delete[] m_pcHand;
            m_pcHand = NULL;
        }

        

        if (m_pcJointControl != NULL)
        {
            delete m_pcJointControl;
            m_pcJointControl = NULL;
        }
        if (m_pcRLControl != NULL)
        {
            delete m_pcRLControl;
            m_pcRLControl = NULL;
        }
        if (m_pcMimicRLControl != NULL)
        {
            delete m_pcMimicRLControl;
            m_pcMimicRLControl = NULL;
        }
       


        return TRUE;
    }


    
    return FALSE;
}

void CHumanoid::OnRecvROS2Msg(PVOID apMsg, PVOID apPlaceholder0, PVOID apPlaceholder1, PVOID apPlaceholder2)
{
    TSTRING strMsg = *(TSTRING*)apMsg;
 
    

    DBG_LOG_TRACE("Received ROS2: %s", strMsg.c_str());
}

void CHumanoid::OnRecvROS2Imu(PVOID apMsg, PVOID apPlaceholder0, PVOID apPlaceholder1, PVOID apPlaceholder2)
{
    // DBG_LOG_TRACE("ROS2 Received:IMU");
    auto imu_ptr = *static_cast<std::shared_ptr<sensor_msgs::msg::Imu>*>(apMsg);
     
    if (!imu_ptr) {
        DBG_LOG_TRACE("NULL IMU");
        return;
    }

    // m_ImuQueue.try_enqueue(*imu_ptr);
    m_ImuData = *imu_ptr;
 
    

    // DBG_LOG_TRACE("Receive IMU:: Orientation -> x: %.2f, y: %.2f, z: %.2f, w: %.2f",
    //         imu_ptr->orientation.x, imu_ptr->orientation.y, imu_ptr->orientation.z, imu_ptr->orientation.w);
}

sensor_msgs::msg::Imu CHumanoid::GetImuData()
{
    sensor_msgs::msg::Imu imu_data;
    // if (m_ImuQueue.try_dequeue(imu_data)) {
    //     m_ImuData = imu_data;
    // }


    return m_ImuData;
}


void CHumanoid::OnRecvROS2PointCloud(PVOID apMsg, PVOID apPlaceholder0, PVOID apPlaceholder1, PVOID apPlaceholder2)
{
    
    // DBG_LOG_TRACE("Enter PointCloud");
    
    auto point_cloud_ptr = *static_cast<std::shared_ptr<sensor_msgs::msg::PointCloud2>*>(apMsg);

    // DBG_LOG_TRACE("Enter PointCloud width = %d, height = %d, point step=%d, field size=%d", point_cloud_ptr->width, point_cloud_ptr->height, point_cloud_ptr->point_step, point_cloud_ptr->fields.size());

    if (!point_cloud_ptr) {
        DBG_LOG_TRACE("NULL PointCloud");
        return;
    }

    m_PointCloudData = *point_cloud_ptr;

    auto compressed_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(*point_cloud_ptr);

    // 원본 데이터 크기 저장
    size_t original_size = point_cloud_ptr->data.size();
    
    // 압축 버퍼의 최대 크기 계산
    size_t max_dst_size = ZSTD_compressBound(original_size);
    std::vector<uint8_t> compressed_data(max_dst_size);

    size_t compressed_size = ZSTD_compress(
                compressed_data.data(), 
                max_dst_size, 
                point_cloud_ptr->data.data(), 
                original_size, 
                3  // 압축 레벨 (1-22, 값이 클수록 압축률 증가, 속도 감소)
                );

    if (ZSTD_isError(compressed_size)) {
        DBG_LOG_TRACE("압축 오류: %s", ZSTD_getErrorName(compressed_size));
        return;
    }

    // 압축된 데이터로 메시지 업데이트
    compressed_data.resize(compressed_size);
    compressed_msg->data = compressed_data;

    double ratio = static_cast<double>(original_size) / compressed_size;

    m_pcPublisher_PointCloud->SetMessage(compressed_msg);

    
    DBG_LOG_TRACE( 
            "PointCloud 압축: 원본 %zu 바이트 → 압축 %zu 바이트 (압축률: %.2f배)",
            original_size, compressed_size, ratio);

    


    // size_t original_size = point_cloud_ptr->data.size();
    // size_t compressed_bound = ZSTD_compressBound(original_size);

    // std::vector<uint8_t> compressed_data(sizeof(original_size) + compressed_bound);

    // // 원본 크기 저장 (압축 해제 시 필요)
    // memcpy(compressed_data.data(), &original_size, sizeof(original_size));

    // // Zstd 압축 실행
    // size_t compressed_size = ZSTD_compress(
    //     compressed_data.data() + sizeof(original_size),
    //     compressed_bound,
    //     point_cloud_ptr->data.data(),
    //     original_size,
    //     3  // 압축 레벨 (1~22, 기본값 3)
    // );

    // if (ZSTD_isError(compressed_size)) {
    //     DBG_LOG_TRACE("Zstd Compression Failed: %s", ZSTD_getErrorName(compressed_size));
    //     return;
    // }

    // if (compressed_size == 0 || compressed_size > compressed_bound) {
    //     DBG_LOG_TRACE("Unexpected compressed size: %zu (expected max: %zu)", compressed_size, compressed_bound);
    //     return;
    // }

    // compressed_data.resize(sizeof(original_size) + compressed_size);

    // auto compressed_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    // *compressed_msg = *point_cloud_ptr;
    // compressed_msg->data.assign(compressed_data.begin(), compressed_data.end());

    // m_pcPublisher_PointCloud->SetMessage(compressed_msg);

    // DBG_LOG_TRACE("Received ROS2: PointCloud");

}

void CHumanoid::OnRecvROS2AxisInfoCmd(PVOID apMsg, PVOID apPlaceholder0, PVOID apPlaceholder1, PVOID apPlaceholder2) 
{
    auto axis_cmd_ptr = *static_cast<std::shared_ptr<axis_info::msg::AxisCmd>*>(apMsg);
    if (!axis_cmd_ptr) {
        DBG_LOG_TRACE("NULL AxisInfo:AxisCmd");
        return;
    }

    // // std::cout << "[Recv ROS2 AxisCmd]" << std::endl;
    
    // const auto& hand_positions = axis_cmd_ptr->target_positions_hands;
    
    // if (!hand_positions.empty()) 
    // {
    //     std::vector<double> current_joints;
    //     current_joints.reserve(20);

    //     int joint_count = (hand_positions.size() < 20) ? hand_positions.size() : 20;

    //     for (int joint = 0 ; joint < joint_count ; joint++) 
    //     {
    //         current_joints.push_back(double(hand_positions[joint]));
    //     }

    //     // Thread safely enqueue asynchronous hand target positions (Hyesung)
    //     {
    //         std::lock_guard<std::mutex> lock(m_mtxRightHandTargetPos);
    //         m_qRightHandTargetPos.push(current_joints);
            
    //         // MAX QUEUE SIZE = 10
    //         if (m_qRightHandTargetPos.size() > 10) 
    //         {
    //             m_qRightHandTargetPos.pop();
    //         }
    //     }
    // } 
    // else 
    // {
    //     std::cout << "Right Hand Target Positions: [Empty]" << std::endl;
    // }
    
    // std::cout << "===========================" << std::endl;

    {
        std::lock_guard<std::mutex> lock(m_mtxAxisCmd);
        if (m_qAxisCmd.size() >= MAX_AXIS_CMD_QUEUE_SIZE) {
            m_qAxisCmd.pop();
        }
        m_qAxisCmd.push(*axis_cmd_ptr);
    }
}

BOOL
CHumanoid::InitEtherCAT()
{
    // Initialize both EtherCAT masters with their respective IDs
    for (int i = 0; i < NUM_OF_MASTER; i++) {
        m_pcEcatMaster[i] = new CEcatMaster(i);        
    }
    

    INT32 nTotalSlaves = m_pcConfigRobot->GetEcatSlaveList().size();
    VEC_ECAT_SLAVE_CONF vSlaveInfoList = m_pcConfigRobot->GetEcatSlaveList();
    
        
    if (nTotalSlaves > 0) {        
        m_pcEcatElmo = new CAxisELMO* [nTotalSlaves-NUM_OF_MASTER];

        m_pcTerminals = new CSlaveBeckhoffCU1124* [NUM_OF_MASTER];
    }

    VEC_HAND_ECAT_SLAVE_CONF vHandSlaveInfoList = m_pcConfigRobot->GetHandEcatSlaveList();
    INT32 nHandSlaves = m_pcConfigRobot->GetEcatMasterConf().nNoOfHands;
    if (nHandSlaves > 0)
        m_pcHand = new CHand* [nHandSlaves];
    

    INT16 nElmoCnt = 0;
    INT16 nJunctionCnt = 0;
    INT16 nGearCnt = 0;
    INT16 nHandCnt = 0;
    INT16 nSlaveCnt = 0;
    
    // Map to track quartet slaves by group ID
    std::map<INT32, std::shared_ptr<CElmoQuartet>> quartetSlaves;
        
        /* Read */
    for (int nCnt = 0; nCnt < (nTotalSlaves+nHandSlaves); nCnt++)
    {

        if (nCnt == 28 || nCnt == 36) { //8,16   13.21,     28,36
            ST_HAND_CONFIG_ECAT_SLAVE stHand = vHandSlaveInfoList[nHandCnt];

            m_pcHand[nHandCnt] = new CHand(stHand.nNoAxes, stHand.nNoJoints, stHand.bEnabled, stHand.bConnected);
            m_pcHand[nHandCnt]->SetName(stHand.strName);
            m_pcHand[nHandCnt]->SetVendorInfo(stHand.uVendorID, stHand.uProductCode);
            m_pcHand[nHandCnt]->SetDCInfo(stHand.bDCSupported, stHand.usDCActivateWord, stHand.nDCSyncShift);

            for (int j=0; j < stHand.nNoJoints; j++) {
                m_pcHand[nHandCnt]->SetPositionLimits(j, stHand.dPosLimitL[j], stHand.dPosLimitU[j]);                    
            }

            // For now, hands are assigned to master 0 (add MASTER_ID to hand config later if needed)
            if (FALSE == m_pcHand[nHandCnt]->Init(*m_pcEcatMaster[stHand.nMasterID]))
            {
                DBG_LOG_ERROR("(%s) Cannot Initialize Hand Slave No. %d", "CHumanoid", nHandCnt);
                return FALSE;
            }
            AddHand(m_pcHand[nHandCnt]);
            nHandCnt += 1;
        }
        else {
            
            ST_CONFIG_ECAT_SLAVE stSlaveInfo = vSlaveInfoList[nSlaveCnt];
            
            if (stSlaveInfo.strName.find("Junction") != std::string::npos) {
                m_pcTerminals[nJunctionCnt] = new CSlaveBeckhoffCU1124();
                
                // Add terminal to the correct master based on MASTER_ID
                INT32 masterID = stSlaveInfo.nMasterID;
                if (masterID >= 0 && masterID < 2) {
                    m_pcEcatMaster[masterID]->AddSlave(m_pcTerminals[nJunctionCnt]);
                    nJunctionCnt++;
                    if (masterID == 1) {
                        nGearCnt = 0;
                    }
                    DBG_LOG_INFO("(%s) Added terminal %s to master %d", "CHumanoid", stSlaveInfo.strName.c_str(), masterID);
                } else {
                    DBG_LOG_ERROR("(%s) Invalid master ID %d for terminal %s", "CHumanoid", masterID, stSlaveInfo.strName.c_str());
                    return FALSE;
                }
            }
            else {
                eAxisType ejointtemp;
                if (stSlaveInfo.nJointType >= eAxisRevolute && stSlaveInfo.nJointType <= eAxisJointless)
                    ejointtemp = eAxisType(stSlaveInfo.nJointType);
                else
                    ejointtemp = eAxisRevolute;

            // Check if this is a quartet axis
            if (stSlaveInfo.bIsQuartet) {
                // Create or get existing quartet slave for this group
                std::shared_ptr<CElmoQuartet> quartetSlave;
                
                if (quartetSlaves.find(stSlaveInfo.nQuartetID) == quartetSlaves.end()) {
                    // First axis of this quartet group - create the quartet slave
                    quartetSlave = std::make_shared<CElmoQuartet>();
                    quartetSlave->SetVendorInfo(stSlaveInfo.uVendorID, stSlaveInfo.uProductCode);
                    quartetSlave->SetDCInfo(stSlaveInfo.bDCSupported, stSlaveInfo.usDCActivateWord, stSlaveInfo.nDCSyncShift);
                    
                    // Add quartet slave to the correct master based on MASTER_ID
                    INT32 masterID = stSlaveInfo.nMasterID;
                    if (masterID >= 0 && masterID < 2) {
                        m_pcEcatMaster[masterID]->AddSlave(quartetSlave.get());
                        quartetSlaves[stSlaveInfo.nQuartetID] = quartetSlave;
                        DBG_LOG_INFO("(%s) Created quartet slave for group %d on master %d", "CHumanoid", stSlaveInfo.nQuartetID, masterID);
                    } else {
                        DBG_LOG_ERROR("(%s) Invalid master ID %d for quartet group %d", "CHumanoid", masterID, stSlaveInfo.nQuartetID);
                        return FALSE;
                    }
                } else {
                    // Get existing quartet slave
                    quartetSlave = quartetSlaves[stSlaveInfo.nQuartetID];
                }
                
                // Create quartet axis using the special constructor
                m_pcEcatElmo[nElmoCnt] = new CAxisELMO(ejointtemp, stSlaveInfo.dEncResolution, stSlaveInfo.dAbsEncResolution,
                                                      stSlaveInfo.dGearRatio, stSlaveInfo.dTransRatio, 
                                                      stSlaveInfo.nQuartetIdx,  // Axis index (0,1,2,3)
                                                      stSlaveInfo.bAbsEnc, stSlaveInfo.bCCW, 
                                                      stSlaveInfo.bEnabled, stSlaveInfo.bConnected);

                
                
                // Associate this axis with the quartet slave
                m_pcEcatElmo[nElmoCnt]->SetQuartetSlave(quartetSlave);
                
                DBG_LOG_INFO("(%s) Created quartet axis %d for group %d", "CHumanoid", 
                           stSlaveInfo.nQuartetIdx, stSlaveInfo.nQuartetID);
            }
            else {
                // Create single axis (existing code)
                m_pcEcatElmo[nElmoCnt] = new CAxisELMO(ejointtemp, stSlaveInfo.dEncResolution, stSlaveInfo.dAbsEncResolution,
                                                      stSlaveInfo.dGearRatio, stSlaveInfo.dTransRatio, 
                                                      stSlaveInfo.bAbsEnc, stSlaveInfo.bCCW, 
                                                      stSlaveInfo.bEnabled, stSlaveInfo.bConnected);

                // Configure single axis EtherCAT settings
                m_pcEcatElmo[nElmoCnt]->SetVendorInfo(stSlaveInfo.uVendorID, stSlaveInfo.uProductCode);
                m_pcEcatElmo[nElmoCnt]->SetDCInfo(stSlaveInfo.bDCSupported, stSlaveInfo.usDCActivateWord, stSlaveInfo.nDCSyncShift);
            }

            // Common configuration for both single and quartet axes
            
            m_pcEcatElmo[nElmoCnt]->SetName(stSlaveInfo.strName);
            m_mapAxisNameToIdx[stSlaveInfo.strName] = nElmoCnt;
            DBG_LOG_INFO("(%s) Enumerated Axis %d: %s", "CHumanoid", nElmoCnt, stSlaveInfo.strName.c_str());
            m_pcEcatElmo[nElmoCnt]->SetOneTurnRef(stSlaveInfo.dOneTurnRef);

                /* Set physical limits */
            m_pcEcatElmo[nElmoCnt]->SetVelocityLimits(ConvertRpm2Rad(stSlaveInfo.dVelLimitL), ConvertRpm2Rad(stSlaveInfo.dVelLimitU));
            m_pcEcatElmo[nElmoCnt]->SetAccelerationLimits(stSlaveInfo.dAccLimitL, stSlaveInfo.dAccLimitU);
            m_pcEcatElmo[nElmoCnt]->SetDecelerationLimits(stSlaveInfo.dDecLimitL, stSlaveInfo.dDecLimitU);
            m_pcEcatElmo[nElmoCnt]->SetJerkLimits(stSlaveInfo.dJerkLimitL, stSlaveInfo.dJerkLimitU);
            m_pcEcatElmo[nElmoCnt]->SetTorqueLimits(stSlaveInfo.dTorLimitL, stSlaveInfo.dTorLimitU);
            m_pcEcatElmo[nElmoCnt]->SetPositionLimits(stSlaveInfo.dPosLimitL, stSlaveInfo.dPosLimitU);

            m_pcEcatElmo[nElmoCnt]->SetRatedTorque(stSlaveInfo.dRatedTorque);
            m_pcEcatElmo[nElmoCnt]->SetRatedCurrent(stSlaveInfo.dRatedCurrent);

            m_pcEcatElmo[nElmoCnt]->SetPGain(stSlaveInfo.dPGain);
            m_pcEcatElmo[nElmoCnt]->SetDGain(stSlaveInfo.dDGain);
                
                /* Set Position at startup, compensate position for incremental encoder */
            m_pcEcatElmo[nElmoCnt]->SetPositionBeforeExit(stSlaveInfo.nPosBeforeExit);

            INT32 masterID = stSlaveInfo.nMasterID;
            // if (masterID >= 0 && masterID < 2) {                     
            //     // m_pcEcatMaster[masterID]->AddSlave(m_pcEcatElmo[nElmoCnt]);
            //     m_pcEcatMaster[masterID]->m_dGearRatio[nGearCnt] = stSlaveInfo.dGearRatio;
            //     m_pcEcatMaster[masterID]->m_dResolution[nGearCnt] = stSlaveInfo.dEncResolution;
            //     m_pcEcatMaster[masterID]->m_dAbsResolution[nGearCnt] = stSlaveInfo.dAbsEncResolution;
            //     nGearCnt++;
            // }

            m_pcEcatElmo[nElmoCnt]->SetMasterID(masterID);
                


            /* determine homing method */
            eHomingMethod ehomingtemp;
            if (stSlaveInfo.nHomingMethod > (INT32)eAxisHomingNotSet && stSlaveInfo.nHomingMethod <= (INT32)eAxisHomeBuiltin)
                ehomingtemp = (eHomingMethod)stSlaveInfo.nHomingMethod;
            else
                ehomingtemp = eAxisHomeStartPos;

            m_pcEcatElmo[nElmoCnt]->SetHomingMethod(ehomingtemp);

            if (eAxisHomeSearch == ehomingtemp)
                m_pcEcatElmo[nElmoCnt]->SetHomeReference(stSlaveInfo.dHomeSearchRef);
            else if (eAxisHomeManual == ehomingtemp)
                m_pcEcatElmo[nElmoCnt]->SetHomePosition(stSlaveInfo.nHomePositionOffset);

            /* Set operational motion parameters
            * Limits should be configured first.
            */
            if (FALSE == m_pcEcatElmo[nElmoCnt]->SetVelocity(ConvertRpm2Rad(stSlaveInfo.dOperatingVel)))
            {
                DBG_LOG_WARN("(%s) Cannot Set Velocity to %lf for Slave No. %d", "CHumanoid", stSlaveInfo.dOperatingVel ,nElmoCnt);
            }

            if (FALSE == m_pcEcatElmo[nElmoCnt]->SetAcceleration(stSlaveInfo.dOperatingAcc))
            {
                DBG_LOG_WARN("(%s) Cannot Set Acceleration to %lf for Slave No. %d", "CHumanoid", stSlaveInfo.dOperatingAcc, nElmoCnt);
            }

            if (FALSE == m_pcEcatElmo[nElmoCnt]->SetDeceleration(stSlaveInfo.dOperatingDec))
            {
                DBG_LOG_WARN("(%s) Cannot Set Deceleration to %lf for Slave No. %d", "CHumanoid", stSlaveInfo.dOperatingDec, nElmoCnt);
            }

            // Initialize axis with the correct master based on MASTER_ID
            masterID = stSlaveInfo.nMasterID;
            if (masterID >= 0 && masterID < 2) {
                if (FALSE == m_pcEcatElmo[nElmoCnt]->Init(*m_pcEcatMaster[masterID], (INT8)stSlaveInfo.nDriveMode))
                {
                    DBG_LOG_ERROR("(%s) Cannot Initialize Slave No. %d on master %d", "CHumanoid", nCnt, masterID);
                    return FALSE;
                }
            } else {
                DBG_LOG_ERROR("(%s) Invalid master ID %d for axis %d", "CHumanoid", masterID, nElmoCnt);
                return FALSE;
            }            
            AddAxis(m_pcEcatElmo[nElmoCnt]);
            nElmoCnt++;
        }

            nSlaveCnt += 1;
        }
             
    }

        
    
    
        
    // for (int i = 0; i < nHandSlaves; i++) {

    //     ST_HAND_CONFIG_ECAT_SLAVE stHand = vHandSlaveInfoList[i];

    //     m_pcHand[i] = new CHand(stHand.nNoAxes, stHand.nNoJoints, stHand.bEnabled, stHand.bConnected);
    //     m_pcHand[i]->SetName(stHand.strName);
    //     m_pcHand[i]->SetVendorInfo(stHand.uVendorID, stHand.uProductCode);
    //     m_pcHand[i]->SetDCInfo(stHand.bDCSupported, stHand.usDCActivateWord, stHand.nDCSyncShift);

    //     for (int j=0; j < stHand.nNoJoints; j++) {
    //         m_pcHand[i]->SetPositionLimits(i, stHand.dPosLimitL[j], stHand.dPosLimitU[j]);                    
    //     }

    //     // For now, hands are assigned to master 0 (add MASTER_ID to hand config later if needed)
    //     if (FALSE == m_pcHand[i]->Init(*m_pcEcatMaster[stHand.nMasterID]))
    //     {
    //         DBG_LOG_ERROR("(%s) Cannot Initialize Hand Slave No. %d", "CHumanoid", i);
    //         return FALSE;
    //     }
    //     AddHand(m_pcHand[i]);

    // }

    ST_CONFIG_ECAT_MASTER stEcatMaster = m_pcConfigRobot->GetEcatMasterConf();
    m_nEcatCycle = stEcatMaster.nCycleTime;
    
    // Initialize both EtherCAT masters
    for (int masterIdx = 0; masterIdx < NUM_OF_MASTER; masterIdx++)
        {
        if (FALSE == m_pcEcatMaster[masterIdx]->Init(stEcatMaster.nCycleTime, stEcatMaster.bDCEnabled))
        {
            DBG_LOG_ERROR("(%s) Cannot Initialize EtherCAT Master %d!", "CHumanoid", masterIdx);
            return FALSE;
        }
        DBG_LOG_INFO("(%s) EtherCAT Master %d initialized successfully", "CHumanoid", masterIdx);
    }

    

    // if (!m_pcEcatMaster[0]->VerifyAllSlavesMapping()) {
    //     DBG_LOG_WARN("(%s) Some slaves need remapping, attempting automatic remap...", "CEcatMasterBase");
    //     if (!m_pcEcatMaster[0]->RemapAllSlaves(3)) {
    //         DBG_LOG_ERROR("(%s) PDO remapping failed for some slaves!", "CEcatMasterBase");
    //         // Continue anyway - some slaves might still work
    //     }
    // // }

    // if (!m_pcEcatMaster[1]->VerifyAllSlavesMapping()) {
    //     DBG_LOG_WARN("(%s) Some slaves need remapping, attempting automatic remap...", "CEcatMasterBase");
    //     if (!m_pcEcatMaster[1]->RemapAllSlaves(3)) {
    //         DBG_LOG_ERROR("(%s) PDO remapping failed for some slaves!", "CEcatMasterBase");
    //         // Continue anyway - some slaves might still work
    //     }
    // }

    // struct timespec ts = {.tv_sec=0, .tv_nsec=(long)1000*1000};
    // for (int i=0; i<3; i++) {
    //     ecrt_master_receive(m_pcEcatMaster[0]->m_stEcatMasterDesc.pEcatMaster);        
    //     ecrt_domain_process(m_pcEcatMaster[0]->m_stEcatMasterDesc.pEcatDomainIn);
    //     ecrt_domain_process(m_pcEcatMaster[0]->m_stEcatMasterDesc.pEcatDomainOut);
    //     ecrt_domain_queue(m_pcEcatMaster[0]->m_stEcatMasterDesc.pEcatDomainIn);
    //     ecrt_domain_queue(m_pcEcatMaster[0]->m_stEcatMasterDesc.pEcatDomainOut);
    //     ecrt_master_send(m_pcEcatMaster[0]->m_stEcatMasterDesc.pEcatMaster);

    //     ecrt_master_receive(m_pcEcatMaster[1]->m_stEcatMasterDesc.pEcatMaster);        
    //     ecrt_domain_process(m_pcEcatMaster[1]->m_stEcatMasterDesc.pEcatDomainIn);
    //     ecrt_domain_process(m_pcEcatMaster[1]->m_stEcatMasterDesc.pEcatDomainOut);
    //     ecrt_domain_queue(m_pcEcatMaster[1]->m_stEcatMasterDesc.pEcatDomainIn);
    //     ecrt_domain_queue(m_pcEcatMaster[1]->m_stEcatMasterDesc.pEcatDomainOut);
    //     ecrt_master_send(m_pcEcatMaster[1]->m_stEcatMasterDesc.pEcatMaster);

    //     nanosleep(&ts, NULL);
    // }


	return TRUE;
}

BOOL
CHumanoid::InitConfig()
{
    return TRUE;
}

void
CHumanoid::DoAgingTest()
{
    // const int nTotalSlaves =  16;
    
    // static double dTimeInterval[nTotalSlaves] = {0., };
    // double dTargetPos = 0.;
    // m_dAgingFreq = 0.25;
    
    // for (int nMotorCnt = 0; nMotorCnt < (int)GetTotalEcatSlaves(); nMotorCnt++)
    // {
    //     // consider 16 axis at most
    //     if ((nMotorCnt == 0) || (nMotorCnt == 8))
    //     {
    //         dTargetPos = 0.3 * M_PI * sin(TWO_M_PI * m_dAgingFreq * dTimeInterval[nMotorCnt]);
    //     }
    //     if ((nMotorCnt == 1) || (nMotorCnt == 9))
    //     {
    //         dTargetPos = 10 * sin(TWO_M_PI * m_dAgingFreq * dTimeInterval[nMotorCnt]);
    //     }
    //     else if ((nMotorCnt == 2) || (nMotorCnt == 3) || (nMotorCnt == 10) || (nMotorCnt == 11) )
    //     {
    //         dTargetPos = 10 * sin(TWO_M_PI * m_dAgingFreq * dTimeInterval[nMotorCnt]);
    //     }
    //     else if ((nMotorCnt >= 4 && nMotorCnt < 8) || nMotorCnt >=12)
    //     {
    //         dTargetPos = 0.9 * M_PI * sin(TWO_M_PI * m_dAgingFreq * dTimeInterval[nMotorCnt]);
    //     }
    //     if (TRUE == m_pcEcatEpos4[nMotorCnt]->MoveAxis(dTargetPos, TRUE))
    //     {
    //         dTimeInterval[nMotorCnt] += (double)(1. / (NANOSEC_PER_SEC / m_nEcatCycle));
    //     }
    // }
}



void
CHumanoid::DoInput(double time)
{

    if (m_cKeyPress == ' ')
        return;
        
    switch (m_cKeyPress)
    {
    case '0': // 초기 부팅
        m_unTask = 0; 
        
        break;
    case '1':
        m_unTask = 1;
        break;
    case '2':
        m_unTask = 2;
        break;
    case '3':
        m_unTask = 3;
        break;
    case '4':
        m_unTask = 4;
        break;
    case '5':
        m_unTask = 5;
        break;
    case '6':
        m_unTask = 6;
        break;
    case '7':
        m_unTask = 7;
        break;
    case '8':
        m_unTask = 8;
        break;
    case '9':
        m_unTask = 9;
        break;

    case 'h':
    case 'H':
        DBG_LOG_INFO("Homing Started");
        m_eControlState = HOMING;

        for (int i = 0; i < GetTotalHands(); i++) {
            m_pcHand[i]->SetServoOn();
        }

        // for (int nMotorCnt = 0; nMotorCnt < (int)GetTotalAxis(); nMotorCnt++)
        // {
            
        //     m_pcEcatElmo[nMotorCnt]->ChangeDriveMode(CIA402_PROFILE_POSITION);
        // }
        // for (int nMotorCnt = 0; nMotorCnt < (int)GetTotalAxis(); nMotorCnt++)
        // {

        //     m_pcEcatElmo[nMotorCnt]->MoveAxis(0.0);
        // }
        break;

    case 'c':
    case 'C':
        DBG_LOG_INFO("Control Started");
        m_eControlState = CONTROL;
        break;

    case 'm':
    case 'M': // Move Handif
        m_isHandMove = TRUE;
        m_nMotionPlayIndex = 0;
        // if ((m_pcHand[0]->GetServoStatus() == -1) && (m_pcHand[0]->GetControlMode() == eHAND_POSITION)) {
        //     m_pcHand[0]->MovePosition(10, ConvertDeg2Rad(30.0));
        // }
        break;

    case 'd':
    case 'D':
        DBG_LOG_INFO("Standstill");
        m_eControlState = STANDSTILL;
        break;

    case 'e':
    case 'E':
        
        // for (int nMotorCnt = 0; nMotorCnt < (int)GetTotalAxis(); nMotorCnt++)
        // {
        //     m_pcEcatElmo[nMotorCnt]->EmgStopAxis();
        // }

        DBG_LOG_INFO("EMG");
        break;
    case 'x':
    case 'X':
        
        
        break;

    case 'o':
    case 'O':
        DBG_LOG_INFO("SET OFFSET");
        m_eControlState = OFFSET;
        // for (int i=0; i < (int)GetTotalAxis(); i++) {
        //     m_pcEcatElmo[i]->UpdateHomeOffset();
        // }   
        // m_pcEcatElmo[4]->UpdateHomeOffset();
        // m_pcEcatElmo[11]->UpdateHomeOffset();
        break;

    case 'a':
    case 'A':
        break;

    case 't':
    case 'T':
            
        break;

    case 'p':
    case 'P':
        DBG_LOG_INFO("HAND CONTROL");
        m_eControlState = HAND_CONTROL;
        break;

    case 'r':
    case 'R':
        // m_eControlState = RL;
        break;

    case 'j':
    case 'J': 
    
        break;
    
    case 's':
    case 'S':
        // Stop Hand and Homing (Hyesung)
        if ((m_pcHand[0]->GetServoStatus() == -1) && (m_pcHand[0]->GetControlMode() == eHAND_POSITION)) 
        {            
            for (int i=0; i < m_pcHand[0]->GetTotalJoints(); i++) 
            {
                m_pcHand[0]->MovePosition(i, ConvertDeg2Rad(0.0));
            }
        }
        if ((m_pcHand[1]->GetServoStatus() == -1) && (m_pcHand[1]->GetControlMode() == eHAND_POSITION)) 
        {            
            for (int i=0; i < m_pcHand[1]->GetTotalJoints(); i++) 
            {
                m_pcHand[1]->MovePosition(i, ConvertDeg2Rad(0.0));
            }
        }
        
        m_isHandMove = FALSE;
        break;

    case 'w':
    case 'W':

        break;

    case 'i':
    case 'I':
        DBG_LOG_INFO("SET INIT");
        m_eControlState = INIT;
        
        for (int i = 0; i < GetTotalHands(); i++) {
            m_pcHand[i]->SetControlMode(eHAND_POSITION);
        }

        break;
        
    default:
        break;
    
    
    }

    DBG_LOG_INFO("Task:%d", m_unTask);
    m_cKeyPress = ' ';

    return;

    
}

void
CHumanoid::WriteDataLog()
{
    for (int nCnt = 0; nCnt < (int)GetTotalAxis(); nCnt++)
    {
        TSTRING strAxisNo;
        sprintf(&strAxisNo[0], "Axis%d", nCnt);
        TSTRING fileName =  m_strDataLog + strAxisNo.c_str() + ".csv";
        FILE* pfFileTiming = fopen(fileName.c_str(), "w");
        double dTimeDuration = 0.;

        LISTINT::iterator itTarPos = m_stDataLog[nCnt].vecTarPos.begin();
        LISTINT::iterator itActPos = m_stDataLog[nCnt].vecActPos.begin();
        LISTINT::iterator itActTor = m_stDataLog[nCnt].vecActTor.begin();
        LISTULONG::iterator itTimeStamp = m_stDataLog[nCnt].vecTimestamp.begin();
    
        if ((int)m_stDataLog[nCnt].vecActPos.size() == (int)m_stDataLog[nCnt].vecTarPos.size())
        {
        
            for (; itTarPos != m_stDataLog[nCnt].vecTarPos.end(); itTarPos++, itActPos++, itActTor++, itTimeStamp++)
            {
                dTimeDuration += 0.001;
                fprintf(pfFileTiming, "%lf %ld %d %d %d\n", dTimeDuration, *itTimeStamp, *itTarPos, *itActPos, *itActTor);
            }
        }
        else
        {
            DBG_LOG_WARN("[%s] Cannot write datalog for Axis %d, ActPos:%d, TarPos:%d, ActTor:%d, TS:%d", "CHumanoid", nCnt, m_stDataLog[nCnt].vecActPos.size(),
                m_stDataLog[nCnt].vecTarPos.size(), m_stDataLog[nCnt].vecActTor.size(), m_stDataLog[nCnt].vecTimestamp.size());
            
            continue;
        }
        fclose(pfFileTiming);
    }
}

void CHumanoid::SetTargetTorque(double *dTarget)
{
    // BOOL bFlag = TRUE;
    // for (int nCnt = 0; nCnt < (int)GetTotalAxis(); nCnt++) {
    //     bFlag = m_pcEcatElmo[nCnt]->IsAllowablePosition(m_pcEcatElmo[nCnt]->GetCurrentPos());
    //     if (bFlag == FALSE) {
    //         DBG_LOG_WARN("Axis: %d, Exceeded Position Limit!", nCnt);
    //         m_eControlState = STANDSTILL;
    //         return;
    //     }

    //     bFlag = m_pcEcatElmo[nCnt]->IsAllowableTorque(dTarget[nCnt]/m_pcEcatElmo[nCnt]->GetGearRatio());
    //     if (bFlag == FALSE) {
    //         DBG_LOG_WARN("Axis: %d, Exceeded Position Limit!", nCnt);
    //         m_eControlState = STANDSTILL;
    //         return;
    //     }

    // }


    for (int nCnt = 0; nCnt < (int)GetTotalAxis(); nCnt++)
    {
        // dTarget[nCnt] = 0.0;

        
        if (nCnt == L_ANKLE1 || nCnt == L_ANKLE2 || nCnt == L_TOE
            || nCnt == R_ANKLE1 || nCnt == R_ANKLE2 || nCnt == R_TOE)
        {
            
            dTarget[nCnt] = 0.0;
        }

        if (m_eControlState == STANDSTILL) {
            
            dTarget[nCnt] = 0.0;    
        }

        // dTarget[nCnt] = 0.0;

        m_pcEcatElmo[nCnt]->MoveTorque(dTarget[nCnt]);        
        
        
    }
}

void CHumanoid::SetHandTargetPositions(double *dTargetPos)
{
    if (GetTotalHands() < 1) {
        return;
    }
    
    if (m_eControlState == HOMING || m_eControlState == INIT)
        return;
    
    if (m_pcHand[0]->GetServoStatus() == -1) {
        for (int j = 0; j < m_pcHand[0]->GetTotalJoints(); j++) {
            m_pcHand[0]->MovePosition(j, dTargetPos[j+33]);
        // cout<< dTargetPos[j+33] << ", ";
        }
    }
    // cout << "Left Hand Target Positions: "<<endl;
    
    // cout << endl;

    // cout << "Right Hand Target Positions: "<<endl;
    if (m_pcHand[1]->GetServoStatus() == -1) {
        for (int j = 0; j < m_pcHand[1]->GetTotalJoints(); j++) {
            m_pcHand[1]->MovePosition(j, dTargetPos[j+53]);
            // cout<< dTargetPos[j+53] << ", ";
        }
    }
    // cout << endl << endl;
}

void CHumanoid::SetJointPDGain()
{
    if (m_pcJointControl != NULL) {m_pcJointControl->m_mKpj.setZero();
        m_pcJointControl->m_mKdj.setZero();
        m_pcJointControl->m_mKpRLj.setZero();
        m_pcJointControl->m_mKdRLj.setZero();
        for (int i = 0; i < (int)GetTotalAxis(); i++) {
            m_pcJointControl->m_mKpj(i,i) = m_pcEcatElmo[i]->GetPGain();
            m_pcJointControl->m_mKdj(i,i) = m_pcEcatElmo[i]->GetDGain();
        }
        for (int i = 0; i < (int)GetTotalAxis(); i++) {
            m_pcJointControl->m_mKpRLj(i,i) = m_pcRLControl-> m_mKpRL(i,i);
            m_pcJointControl->m_mKdRLj(i,i) = m_pcRLControl-> m_mKdRL(i,i);
        }
    }    
}

void 
CHumanoid::SetAxisInfoStateForROS2()
{
    if (m_pcPublisher_AxisInfoState != NULL) {
        rclcpp::Time now = m_pcPublisher_AxisInfoState->get_clock()->now();
    
        m_stAxisState.header.stamp.sec = now.seconds();
        m_stAxisState.header.stamp.nanosec = now.nanoseconds()%1000000000;
        m_stAxisState.header.frame_id = "axis_info";
        
        m_stAxisState.axis_id.clear();
        m_stAxisState.control_modes.clear();
        m_stAxisState.positions.clear();
        m_stAxisState.velocities.clear();
        m_stAxisState.torques.clear();
        m_stAxisState.enabled.clear();

        m_stAxisState.task = m_unTask;
        
        for (int i=0; i < GetTotalAxis(); i++) {         
            m_stAxisState.axis_id.push_back(i);
            m_stAxisState.control_modes.push_back(m_pcEcatElmo[i]->GetDriveMode());
            m_stAxisState.positions.push_back(m_pcEcatElmo[i]->GetCurrentPos());
            m_stAxisState.velocities.push_back(m_pcEcatElmo[i]->GetCurrentVel());
            m_stAxisState.torques.push_back(m_pcEcatElmo[i]->GetCurrentTor());
            m_stAxisState.enabled.push_back(m_pcEcatElmo[i]->IsEnabled());
        }

        m_stAxisState.positions_hands.clear();
        for (int i = 0; i < GetTotalHands(); i++) {
            for (int j = 0; j < m_pcHand[i]->GetTotalJoints(); j++) {
                m_stAxisState.positions_hands.push_back(m_pcHand[i]->GetPosition(j));
            }
        }           
        
        m_pcPublisher_AxisInfoState->m_axis_info_state_msg = m_stAxisState;        
    }
}

bool CHumanoid::LoadMotionDataCSV(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        printf("[Error] Failed to open CSV file: %s\n", filename.c_str());
        return false;
    }

    m_vecRightHandMotionData.clear();
    m_vecLeftHandMotionData.clear();
    m_nMotionPlayIndex = 0;

    std::string line;

    const int LH_DATA_START_INDEX = 19; // CSV Left Hand Data Start Index
    const int RH_DATA_START_INDEX = 39; // CSV Right Hand Data Start Index

    const int reorderMap[20] = {
        4, 3, 2, 1, 0,      // Thumb (5)
        8, 7, 6, 5,         // Index (4) -> 5+3 ~ 5+0
        12, 11, 10, 9,      // Middle (4) -> 9+3 ~ 9+0
        16, 15, 14, 13,     // Ring (4) -> 13+3 ~ 13+0
        19, 18, 17          // Little (3) -> 17+2 ~ 17+0
    };

   while (std::getline(file, line)) 
    {
        std::stringstream ss(line);
        std::string cell;
        std::vector<double> raw_row;
        
        while (std::getline(ss, cell, ',')) {
            try {
                raw_row.push_back(std::stod(cell));
            } catch (...) {
                raw_row.push_back(0.0);
            }
        }

        // =========================================================
        // [Left Hand Read Logic]
        // =========================================================
        {
            std::vector<double> reordered_lh_pos(20);
            for (int i = 0; i < 20; i++) {
                int src_idx = LH_DATA_START_INDEX + reorderMap[i];
                reordered_lh_pos[i] = raw_row[src_idx];
            }

            m_vecLeftHandMotionData.push_back(reordered_lh_pos);
        }

        // =========================================================
        // [Right Hand Read Logic]
        // =========================================================
        {
            std::vector<double> reordered_rh_pos(20);
            for (int i = 0; i < 20; i++) {
                int src_idx = RH_DATA_START_INDEX + reorderMap[i];
                reordered_rh_pos[i] = raw_row[src_idx];
            }
            
            m_vecRightHandMotionData.push_back(reordered_rh_pos);
        }
    }

    file.close();
    
    if (m_vecLeftHandMotionData.size() != m_vecRightHandMotionData.size()) {
        printf("[Warning] Motion Data Size Mismatch! L:%d, R:%d\n", 
               (int)m_vecLeftHandMotionData.size(), (int)m_vecRightHandMotionData.size());
        return false;
    }

    printf("[Info] Motion Data Loaded. Frames: %d\n", (int)m_vecRightHandMotionData.size());
    return true;
}

void
proc_main_control(void* apRobot)
{
	CHumanoid* pRobot = (CHumanoid*)apRobot;
	int nCnt = 0;
    DBG_LOG_INFO("(%s) Main Control Task Started!", "proc_main_control");

    RTTIME tmCurrent = 0, tmPrev = 0, tmTrackS = 0, tmTrackE = 0, tmCurrent2=0;;
    RTTIME tmSend = 0;
    RTTIME tmPeriod = 0, tmResp = 0;
    RTTIME tmMaxPeriod = 0, tmMaxResp = 0;

    RTTIME tmStart = 0;
    tmStart = read_timer();

    double dTargetTorques[NUM_OF_JOINTS] = {0.};

    BOOL bHand = FALSE;
    axis_info::msg::AxisCmd axis_cmd;

    // Interpolation period (33ms) (Hyesung)
    const int INTERP_PERIOD = 33;

    // Csv read period (33ms) (Hyesung)
    const int CSV_READ_PERIOD = 33;
    
    // Joint velocity limit (Hyesung)
    const double VELOCITY_LIMIT_RAD_S = 2.0; // 2 rad/s
    const double CONTROL_PERIOD_SEC = 0.001; // 1 ms
    const double MAX_STEP_RAD = VELOCITY_LIMIT_RAD_S * CONTROL_PERIOD_SEC; // 0.002 rad per tick

	while (!pRobot->CheckStopTask())
	{
        
		wait_next_period(NULL);
        tmCurrent = read_timer();

        double time_sec = (double)(tmCurrent - tmStart) / 1000000000.0;  // us → sec
   
        // run only when EtherCAT master and slaves are in operational state

        if (pRobot->m_pcJointControl->m_bTorqueOff) {
            DBG_LOG_INFO("Robot Task Stop");
            pRobot->StopTasks();
        }

        // pRobot->DoInput(time_sec);
        if ((TRUE == pRobot->m_bEcatOP) && (pRobot->m_bAllServoOn))
        {

            pRobot->PopAxisCmd(axis_cmd);
            for (int nCnt=0; nCnt < pRobot->GetTotalAxis(); nCnt++) {
                
                if (pRobot->m_pcEcatElmo[nCnt]->CheckAxisLimits() == FALSE) {      
                    // printf("Limited::\n");
                    // pRobot->m_pcEcatElmo[nCnt]->StopAxis();                    
                     
                    pRobot->StopTasks();
                    
                }


                pRobot->m_dPositions[nCnt] = pRobot->m_pcEcatElmo[nCnt]->GetCurrentPos();
                pRobot->m_dVelocities[nCnt] = pRobot->m_pcEcatElmo[nCnt]->GetCurrentVel();
                pRobot->m_dAbsPositions[nCnt] = pRobot->m_pcEcatElmo[nCnt]->GetAdditionalPos();
                

            }

            // printf("pitch = %lf, yaw = %lf\n", pRobot->m_AxisCmd.target_positions[NECK_PITCH], pRobot->m_AxisCmd.target_positions[NECK_YAW]);

            // printf("task = %d", pRobot->m_unTask);
            // printf("\n");
            
            pRobot->DoInput(time_sec);

            

            if (pRobot->m_pcJointControl != NULL) {

                pRobot->m_pcJointControl->ConvertStateMotor2Joint(pRobot->m_unTask, pRobot->m_eControlState, time_sec, pRobot->m_dPositions, pRobot->m_dVelocities, pRobot->m_dAbsPositions, pRobot->m_AxisCmd.target_positions[NECK_PITCH], pRobot->m_AxisCmd.target_positions[NECK_YAW]);
        
                pRobot->m_pcJointControl->ConvertCmdJoint2Motor(dTargetTorques);
        
                pRobot->SetTargetTorque(dTargetTorques);
                pRobot->SetHandTargetPositions(dTargetTorques);

            }
            
        }

        
        // /*
        // =========================================================
        // [CSV Motion Data Read] (Hyesung)
        // ========================================================= 
        // Csv Read Rate: 30 Hz (33ms)
        // =========================================================
        // */
        
        // if (pRobot->m_isHandMove && nCnt % CSV_READ_PERIOD == 0) // about 30 Hz => 33ms
        // {
        //     // Set the previous target as the new start point
        //     pRobot->m_vecRightHandStartPos = pRobot->m_vecRightHandCurrentRefPos;
        //     pRobot->m_vecLeftHandStartPos = pRobot->m_vecLeftHandCurrentRefPos;

        //     // Retrieve new target from pre-loaded csv memory
        //     bool bHasNewData = false;
        //     VECDOUBLE nextRightTarget, nextLeftTarget;
            
        //     // Print current motion steps
        //     std::cout << "Motion Play Steps: " << pRobot->m_nMotionPlayIndex << std::endl;

        //     // Check csv 
        //     if (pRobot->m_nMotionPlayIndex < pRobot->m_vecRightHandMotionData.size() && pRobot->m_nMotionPlayIndex < pRobot->m_vecLeftHandMotionData.size()) 
        //     {
        //         nextRightTarget = pRobot->m_vecRightHandMotionData[pRobot->m_nMotionPlayIndex];
        //         nextLeftTarget = pRobot->m_vecLeftHandMotionData[pRobot->m_nMotionPlayIndex];
        //         pRobot->m_nMotionPlayIndex++;
        //         bHasNewData = true;
        //     }
        //     else 
        //     {
        //         DBG_LOG_INFO("Motion Finished!");
        //         pRobot->m_isHandMove = FALSE;
        //     }

        //     // Set new target if available, otherwise hold position
        //     if (bHasNewData && nextRightTarget.size() == 20 && nextLeftTarget.size() == 20)
        //     {
        //         pRobot->m_vecRightHandEndPos = nextRightTarget; // Update
        //         pRobot->m_vecLeftHandEndPos = nextLeftTarget; // Update
        //     } 
        //     else 
        //     {
        //         pRobot->m_vecRightHandEndPos = pRobot->m_vecRightHandStartPos; // Hold
        //         pRobot->m_vecLeftHandEndPos = pRobot->m_vecLeftHandStartPos; // Hold
        //     }
            
        //     // Reset interpolation tick count
        //     pRobot->m_nRightHandInterpTick = 0;
        //     pRobot->m_nLeftHandInterpTick = 0;
        // }   
       
        /*
        =========================================================
        [Teleop Motion Command Update] (Hyesung)
        ========================================================= 
        ROS2 Command Enqueue Rate: Unsynchronous (Variable)
        Command Pop Rate: 30 Hz (33ms)
        =========================================================
        */

        // if (pRobot->m_isHandMove && nCnt % INTERP_PERIOD == 0) // about 30 Hz => 33ms
        // {
        //     // Set the previous target as the new start point
        //     pRobot->m_vecHandStartPos = pRobot->m_vecHandEndPos;
            
        //     // Retrieve new target data from the queue (Thread-safe)
        //     bool bHasNewData = false;
        //     VECDOUBLE nextTarget;

        //     {
        //         std::lock_guard<std::mutex> lock(pRobot->m_mtxRightHandTargetPos);
                
        //         if (!pRobot->m_qRightHandTargetPos.empty()) 
        //         {
        //             nextTarget = pRobot->m_qRightHandTargetPos.front();
        //             pRobot->m_qRightHandTargetPos.pop();
        //             bHasNewData = true;
        //         }
        //     }

        //     // Set new target if available, otherwise hold position
        //     if (bHasNewData && nextTarget.size() == 20) 
        //     {
        //         pRobot->m_vecHandEndPos = nextTarget; // update
        //     } 
        //     else 
        //     {
        //         pRobot->m_vecHandEndPos = pRobot->m_vecHandStartPos; // hold
        //     }
            
        //     // Reset interpolation tick count
        //     pRobot->m_nHandInterpTick = 0;
        // }

        /*


        // =========================================================
        // [Hand Joint Command Interpolation & Velocity Limiting] (Hyesung)
        // ========================================================= 
        // Control Rate: 1 kHz (1ms)
        // =========================================================
        // */

        // // (Right Hand) joint target linear interpolation logic (Hyesung) -> 1ms
        // if (pRobot->m_isHandMove && (pRobot->m_pcHand[0]->GetServoStatus() == -1) && (pRobot->m_pcHand[0]->GetControlMode() == eHAND_POSITION)) {
            
        //     // Calculate alpha for linear interpolation (0.0 ~ 1.0)
        //     double alpha = (double)pRobot->m_nRightHandInterpTick / (double)INTERP_PERIOD;

        //     if (alpha > 1.0) alpha = 1.0;
            
        //     for (int i = 0 ; i < pRobot->m_pcHand[0]->GetTotalJoints() ; i++) 
        //     {
        //         double start = pRobot->m_vecRightHandStartPos[i];
        //         double end = pRobot->m_vecRightHandEndPos[i];

        //         // Ideal joint pos (linear interpolation results)
        //         double ideal_pos = start + (end - start) * alpha;

        //         // Veloicy limit clamping
        //         double current_real_pos = pRobot->m_vecRightHandCurrentRefPos[i];
        //         double diff = ideal_pos - current_real_pos;

        //         if (diff > MAX_STEP_RAD) 
        //         {
        //             diff = MAX_STEP_RAD;
        //         } 
        //         else if (diff < -MAX_STEP_RAD) 
        //         {
        //             diff = -MAX_STEP_RAD;
        //         }

        //         // Define final command position
        //         double final_cmd_pos = current_real_pos + diff;

        //         pRobot->m_vecRightHandCurrentRefPos[i] = final_cmd_pos;

        //         pRobot->m_pcHand[0]->MovePosition(i, final_cmd_pos);
        //     }
            
        //     // Right hand increment tick count
        //     pRobot->m_nRightHandInterpTick++;
        // }

        // // (Left Hand) joint target linear interpolation logic (Hyesung) -> 1ms
        // if (pRobot->m_isHandMove && (pRobot->m_pcHand[1]->GetServoStatus() == -1) && (pRobot->m_pcHand[1]->GetControlMode() == eHAND_POSITION)) {
            
        //     // Calculate alpha for linear interpolation (0.0 ~ 1.0)
        //     double alpha = (double)pRobot->m_nLeftHandInterpTick / (double)INTERP_PERIOD;

        //     if (alpha > 1.0) alpha = 1.0;
            
        //     for (int i = 0 ; i < pRobot->m_pcHand[1]->GetTotalJoints() ; i++) 
        //     {
        //         double start = pRobot->m_vecLeftHandStartPos[i];
        //         double end = pRobot->m_vecLeftHandEndPos[i];

        //         // Ideal joint pos (linear interpolation results)
        //         double ideal_pos = start + (end - start) * alpha;

        //         // Veloicy limit clamping
        //         double current_real_pos = pRobot->m_vecLeftHandCurrentRefPos[i];
        //         double diff = ideal_pos - current_real_pos;

        //         if (diff > MAX_STEP_RAD) 
        //         {
        //             diff = MAX_STEP_RAD;
        //         } 
        //         else if (diff < -MAX_STEP_RAD) 
        //         {
        //             diff = -MAX_STEP_RAD;
        //         }

        //         // Define final command position
        //         double final_cmd_pos = current_real_pos + diff;

        //         pRobot->m_vecLeftHandCurrentRefPos[i] = final_cmd_pos;

        //         pRobot->m_pcHand[1]->MovePosition(i, final_cmd_pos);
        //     }
            
        //     // Left hand increment tick count
        //     pRobot->m_nLeftHandInterpTick++;
        // }

        // if(pRobot->m_isHandMove)
        //     nCnt++;

        // // --------------------------------------------------------- (Hyesung End)


        tmCurrent2 = read_timer();
        double time_sec2 = (double)(tmCurrent2 - tmCurrent) / 1000000000.0;  // us → sec
        if(time_sec2 > 0.0008){
            cout<<"Time err : "<<time_sec2<<endl<<endl<<endl<<endl<<endl<<endl<<endl;
        }
        // 변수 선언 했는데 못 찾아요,,,,
        // pRobot->m_pcJointControl->m_vComputationTime(1) = time_sec2;

        tmPeriod = tmCurrent - tmPrev; 

        // printf("Period: %d.%06d ms\n", tmPeriod / 1000000, tmPeriod % 1000000);
        tmResp = tmTrackE - tmTrackS;

        if (tmPeriod > tmMaxPeriod) tmMaxPeriod = tmPeriod;
        if (tmResp > tmMaxResp) tmMaxResp = tmResp;

        if (!(nCnt++ % 2000)) // print every second
        {
            // DBG_LOG_TRACE("Task Period:%d.%06d ms", tmPeriod / 1000000, tmPeriod % 1000000);
            // DBG_LOG_TRACE("Task Period:%d.%06d ms Task Main Resp:%d.%03d us", tmPeriod / 1000000, tmPeriod % 1000000, tmResp / 1000, tmResp % 1000);
            // DBG_LOG_TRACE("Max Period:%d.%06d ms Max Main Resp:%d.%03d us", tmMaxPeriod / 1000000, tmMaxPeriod % 1000000, tmMaxResp / 1000, tmMaxResp % 1000);
            // DBG_LOG_NOTHING("\n");

            if (pRobot->GetTotalHands() > 0) {
                // if (pRobot->m_eControlState == HAND_CONTROL) {
                //     if ((pRobot->m_pcHand[0]->GetServoStatus() == -1) && (pRobot->m_pcHand[0]->GetControlMode() == eHAND_POSITION)) {
                //         if (bHand) {
                //             for (int i=0; i < pRobot->m_pcHand[0]->GetTotalJoints(); i++) {
                //                 if (i==8 || i== 12 || i==16 || i==19)
                //                     pRobot->m_pcHand[0]->MovePosition(i, ConvertDeg2Rad(0.0));
                //                 else
                //                     pRobot->m_pcHand[0]->MovePosition(i, ConvertDeg2Rad(2.0));
                //             }
                //             bHand = FALSE;
                //         }
                //         else {
                //             for (int i=0; i < pRobot->m_pcHand[0]->GetTotalJoints(); i++) {
                //                 if (i==8 || i== 12 || i==16 || i==19)
                //                     pRobot->m_pcHand[0]->MovePosition(i, ConvertDeg2Rad(2.0));
                //                 else
                //                     pRobot->m_pcHand[0]->MovePosition(i, ConvertDeg2Rad(30.0));
                //             }
                //             bHand = TRUE;
                //         }
                        

                //     }
            
                // }
            }

        }

        tmPrev = tmCurrent;

        
	}    

    DBG_LOG_WARN("[%s]TASK ENDED!", "proc_main_control");
}

void
proc_ethercat_control(void* apRobot)
{
    CHumanoid* pRobot = (CHumanoid*)apRobot;
    int nCnt = 0;

    RTTIME tmCurrent = 0, tmPrev = 0, tmEcat = 0;
    RTTIME tmSend = 0;
    RTTIME tmPeriod = 0, tmResp = 0;
    RTTIME tmMaxPeriod = 0, tmMaxResp = 0;
    RTTIME tmLastAxisInfoState = 0; 

    DBG_LOG_INFO("(%s) EtherCAT Control Task Started!", "proc_ethercat_control");

    std::ofstream log_file("jitter_log.csv");

    int nCleanUp = 0;

    // axis_info::msg::AxisState axis_state;


    pRobot->m_bEcatOP = FALSE;

    while (TRUE)
    {
        wait_next_period(NULL);
        tmCurrent = read_timer();

        

        // Read slaves from both masters
        for (int i = 0; i < NUM_OF_MASTER; i++) {
            pRobot->m_pcEcatMaster[i]->ReadSlaves();
            
        }
        
        // pRobot->UpdateExtInterfaceData();
        

        // Check if both masters and all slaves are operational
        BOOL bMasterOp[2] = {TRUE, TRUE};
        for (int i = 0; i < NUM_OF_MASTER; i++) {
            bMasterOp[i] = (TRUE == pRobot->m_pcEcatMaster[i]->IsAllSlavesOp() && TRUE == pRobot->m_pcEcatMaster[i]->IsMasterOp());
        }
                
        
        if (bMasterOp[0] && bMasterOp[1])        
        {
            pRobot->m_bEcatOP = TRUE;
        }
        else {
            pRobot->m_bEcatOP = FALSE;
        }

        for (int i = 0; i < (int)pRobot->GetTotalAxis(); i++) {
            if (i==0)
                pRobot->m_bAllServoOn = pRobot->m_pcEcatElmo[i]->IsServoOn();
            else
                pRobot->m_bAllServoOn &= pRobot->m_pcEcatElmo[i]->IsServoOn();
        }
        

        // if (pRobot->m_bEcatOP == TRUE) {
        //     BOOL bOnline[2] = {TRUE, };
        //     for (int i = 0; i < NUM_OF_MASTER; i++) {
        //         bOnline[i] = (TRUE == pRobot->m_pcEcatMaster[i]->IsAllSlavesOnline());
        //     }

        //     if ((bOnline[0] == FALSE) || (bOnline[1] == FALSE))
        //         pRobot->StopTasks();
        // }

        // if (pRobot->) {
        //     for (int i = 0; i < pRobot->GetTotalAxis(); i++) {
        //         if (pRobot->m_pcEcatElmo[i]->GetState() == eAxisDisconnected) {
        //             DBG_ERROR("Axis[%d] disconnected", i);
        //             pRobot->StopTasks();
        //         }
        //     }
        // }

        
        
        tmSend = read_timer();
        
        // Write slaves to both masters
        for (int i=0; i< NUM_OF_MASTER; i++)
        {
            pRobot->m_pcEcatMaster[i]->WriteSlaves(tmSend);
        }
        
        
        tmEcat = read_timer();
        // printf("Slave::17 Control Word: %04x, Status Word: %04x\n", pRobot->m_pcEcatElmo[17]->GetControlWord(), pRobot->m_pcEcatElmo[17]->GetStatusWord());
        // printf(" Slave::18 Control Word: %04x, Status Word: %04x\n", pRobot->m_pcEcatElmo[18]->GetControlWord(), pRobot->m_pcEcatElmo[18]->GetStatusWord());
        if (3000 < nCnt)
        {
            tmPeriod = tmCurrent - tmPrev;
            tmResp = tmEcat - tmCurrent;

            if (tmPeriod > tmMaxPeriod) tmMaxPeriod = tmPeriod;
            if (tmResp > tmMaxResp) tmMaxResp = tmResp;

            if (!(nCnt % 1000)) // print every second
            {
                // DBG_LOG_TRACE("Task Period:%d.%06d ms Task Ecat Resp:%d.%03d us", tmPeriod / 1000000, tmPeriod % 1000000, tmResp / 1000, tmResp % 1000);
                // DBG_LOG_TRACE("Max Period:%d.%06d ms Max Ecat Resp:%d.%03d us", tmMaxPeriod / 1000000, tmMaxPeriod % 1000000, tmMaxResp / 1000, tmMaxResp % 1000);
                // DBG_LOG_TRACE("Master State: %02x, Slave State: %02x, DomainStateIn: %02x, DomainStateOut: %02x, No. Of. Slaves: %d", pRobot->m_pcEcatMaster->GetMasterState(), pRobot->m_pcEcatMaster->GetSlaveState(), pRobot->m_pcEcatMaster->GetDomainState(), pRobot->m_pcEcatMaster->GetDomainState(eOutput), pRobot->m_pcEcatMaster->GetRespSlaveNo());
                // DBG_LOG_NOTHING("\n");
            }
        }
        tmPrev = tmCurrent;
        nCnt++;
        
        if (pRobot->CheckStopTask() == TRUE)
        {
            nCleanUp++;
            
            
            for (int nMotorCnt = 0; nMotorCnt < (int)pRobot->GetTotalAxis(); nMotorCnt++)
            {                
                pRobot->m_pcEcatElmo[nMotorCnt]->ServoOff();
            }

            for (int nHandCnt = 0; nHandCnt < pRobot->GetTotalHands(); nHandCnt++) {
                pRobot->m_pcHand[nHandCnt]->SetServoOff();
            }
            if (nCleanUp > 1000)
                break;
        }

        
        // Publish Axis Info State
        if ((tmCurrent - tmLastAxisInfoState) >= 10000000) {
            tmLastAxisInfoState = tmCurrent;
            pRobot->SetAxisInfoStateForROS2();
        }

    
    }

    DBG_LOG_WARN("[%s]TASK ENDED!", "proc_ethercat_control");
}

// void
// proc_ethercat_control(void* apRobot)
// {
//     CHumanoid* pRobot = (CHumanoid*)apRobot;
//     int nCnt = 0;

//     RTTIME tmCurrent = 0, tmPrev = 0, tmEcat = 0;
//     RTTIME tmSend = 0;
//     RTTIME tmPeriod = 0, tmResp = 0;
//     RTTIME tmMaxPeriod = 0, tmMaxResp = 0;

//     DBG_LOG_INFO("(%s) EtherCAT Control Task Started!", "proc_ethercat_control");

//     std::ofstream log_file("jitter_log.csv");

//     int nCleanUp = 0;

//     axis_info::msg::AxisState axis_state;


//     pRobot->m_bEcatOP = FALSE;

//     while (TRUE)
//     {
//         wait_next_period(NULL);
//         tmCurrent = read_timer();

        

//         // Read slaves from both masters
//         for (int i = 0; i < NUM_OF_MASTER; i++) {
//             pRobot->m_pcEcatMaster[i]->ReadSlaves();
            
//         }
        
//         // pRobot->UpdateExtInterfaceData();
        

//         // Check if both masters and all slaves are operational
//         BOOL bMasterOp = TRUE;
        
//             bMasterOp = (TRUE == pRobot->m_pcEcatMaster[0]->IsAllSlavesOp() && TRUE == pRobot->m_pcEcatMaster[0]->IsMasterOp());
        
                
        
//         if (bMasterOp)        
//         {
//             pRobot->m_bEcatOP = TRUE;
//         }
//         else {
//             pRobot->m_bEcatOP = FALSE;
//         }

//         // for (int i = 0; i < (int)pRobot->GetTotalAxis(); i++) {
//         //     if (i==0)
//         //         pRobot->m_bAllServoOn = pRobot->m_pcEcatElmo[i]->IsServoOn();
//         //     else
//         //         pRobot->m_bAllServoOn &= pRobot->m_pcEcatElmo[i]->IsServoOn();
//         // }
        

//         // if (pRobot->m_bEcatOP == TRUE) {
//         //     BOOL bOnline[2] = {TRUE, };
//         //     for (int i = 0; i < NUM_OF_MASTER; i++) {
//         //         bOnline[i] = (TRUE == pRobot->m_pcEcatMaster[i]->IsAllSlavesOnline());
//         //     }

//         //     if ((bOnline[0] == FALSE) || (bOnline[1] == FALSE))
//         //         pRobot->StopTasks();
//         // }

//         // if (pRobot->) {
//         //     for (int i = 0; i < pRobot->GetTotalAxis(); i++) {
//         //         if (pRobot->m_pcEcatElmo[i]->GetState() == eAxisDisconnected) {
//         //             DBG_ERROR("Axis[%d] disconnected", i);
//         //             pRobot->StopTasks();
//         //         }
//         //     }
//         // }

        
        
//         tmSend = read_timer();
        
//         // Write slaves to both masters
        
//             pRobot->m_pcEcatMaster[0]->WriteSlaves(tmSend);
        
        
        
//         tmEcat = read_timer();
//         // printf("Slave::17 Control Word: %04x, Status Word: %04x\n", pRobot->m_pcEcatElmo[17]->GetControlWord(), pRobot->m_pcEcatElmo[17]->GetStatusWord());
//         // printf(" Slave::18 Control Word: %04x, Status Word: %04x\n", pRobot->m_pcEcatElmo[18]->GetControlWord(), pRobot->m_pcEcatElmo[18]->GetStatusWord());
//         if (3000 < nCnt)
//         {
//             tmPeriod = tmCurrent - tmPrev;
//             tmResp = tmEcat - tmCurrent;

//             if (tmPeriod > tmMaxPeriod) tmMaxPeriod = tmPeriod;
//             if (tmResp > tmMaxResp) tmMaxResp = tmResp;

//             if (!(nCnt % 1000)) // print every second
//             {
//                 // DBG_LOG_TRACE("Task Period:%d.%06d ms Task Ecat Resp:%d.%03d us", tmPeriod / 1000000, tmPeriod % 1000000, tmResp / 1000, tmResp % 1000);
//                 // DBG_LOG_TRACE("Max Period:%d.%06d ms Max Ecat Resp:%d.%03d us", tmMaxPeriod / 1000000, tmMaxPeriod % 1000000, tmMaxResp / 1000, tmMaxResp % 1000);
//                 // DBG_LOG_TRACE("Master State: %02x, Slave State: %02x, DomainStateIn: %02x, DomainStateOut: %02x, No. Of. Slaves: %d", pRobot->m_pcEcatMaster->GetMasterState(), pRobot->m_pcEcatMaster->GetSlaveState(), pRobot->m_pcEcatMaster->GetDomainState(), pRobot->m_pcEcatMaster->GetDomainState(eOutput), pRobot->m_pcEcatMaster->GetRespSlaveNo());
//                 // DBG_LOG_NOTHING("\n");
//             }
//         }
//         tmPrev = tmCurrent;
//         nCnt++;
        
//         if (pRobot->CheckStopTask() == TRUE)
//         {
//             nCleanUp++;
            
            
//             for (int nMotorCnt = 0; nMotorCnt < (int)pRobot->GetTotalAxis(); nMotorCnt++)
//             {                
//                 pRobot->m_pcEcatElmo[nMotorCnt]->ServoOff();
//             }

//             for (int nHandCnt = 0; nHandCnt < pRobot->GetTotalHands(); nHandCnt) {
//                 pRobot->m_pcHand[nHandCnt]->SetServoOff();
//             }
//             if (nCleanUp > 1000)
//                 break;
//         }

        

        
//         // rclcpp::Time now = pRobot->m_pcPublisher_AxisInfoState->get_clock()->now();
        
//         // axis_state.header.stamp.sec = now.seconds();
//         // axis_state.header.stamp.nanosec = now.nanoseconds()%1000000000;
//         // axis_state.header.frame_id = "base_link";
        

//         // std::random_device rd;
//         // std::mt19937 gen(rd());
//         // std::uniform_real_distribution<float> dis(0.0f, 1.0f);

//         // axis_state.axis_names.clear();
//         // axis_state.control_modes.clear();
//         // axis_state.positions.clear();
//         // axis_state.velocities.clear();
//         // axis_state.torques.clear();
//         // axis_state.enabled.clear();

//         // // for (int i=0; i < pRobot->m_pcEcatMaster->GetSlaveCnt(); i++) {
//         // for (int i=0; i < pRobot->GetTotalAxis(); i++) {
//         //     // if ((i == 0) || (i==2)) {
//         //     //     continue;
//         //     // }

//         //     // axis_state->axis_names.push_back(pRobot->m_pcEcatElmo[i]->GetName());
//         //     // axis_state->control_modes.push_back(pRobot->m_pcEcatElmo[i]->GetDriveMode());
//         //     // axis_state->positions.push_back(pRobot->m_pcEcatElmo[i]->GetCurrentPos());
//         //     // axis_state->velocities.push_back(pRobot->m_pcEcatElmo[i]->GetCurrentVel());
//         //     // axis_state->torques.push_back(pRobot->m_pcEcatElmo[i]->GetCurrentTor());
//         //     // axis_state->enabled.push_back(pRobot->m_pcEcatElmo[i]->IsEnabled());
//         //     // printf("Publishing Axis Info:%s, position:%f\n", axis_state->axis_names[0].c_str(), axis_state->positions[0]);
//         //     float randomFloat = dis(gen);
//         //     axis_state.axis_names.push_back(std::to_string(i));
//         //     axis_state.control_modes.push_back(i);
//         //     axis_state.positions.push_back(randomFloat);
//         //     axis_state.velocities.push_back(randomFloat);
//         //     axis_state.torques.push_back(randomFloat);
//         //     axis_state.enabled.push_back(true);

            
//         // }

//         // pRobot->m_pcPublisher_AxisInfoState->m_axis_info_state_msg = axis_state;
        

//         // pRobot->m_pcPublisher_AxisInfoState->SetMessage(axis_state, true);



//     }

    

//     DBG_LOG_WARN("[%s]TASK ENDED!", "proc_ethercat_control");
// }



void proc_imu(void* apRobot)
{
    CHumanoid* pRobot = (CHumanoid*)apRobot;

    RTTIME tmStart = 0, tmCurrent=0, tmPre = 0;
    

    DBG_LOG_INFO("(%s) IMU Task Started!", "proc_imu");
    char cKeyPress = ' ';

    // FILE* fp = std::fopen("./Log/imu.csv", "w");
    // fprintf(fp, "time,dt, AccX, AccY, AccZ, GyroX, GyroY, GyroZ, QuatW, QuatX, QuatY, QuatZ\n");
    tmStart = read_timer();
    tmPre = tmStart;

    while (!pRobot->CheckStopTask()) {
        wait_next_period(NULL);


        if (pRobot->m_cIMU.GetImuEnabled())
            pRobot->m_cIMU.ReadData();

        tmCurrent = read_timer();
        
        tmPre = tmCurrent;
    }
    
    DBG_LOG_WARN("[%s]TASK ENDED!", "proc_imu");
    // fclose(fp);
}

void proc_gamepad_control(void* apRobot)
{
    CHumanoid* pRobot = (CHumanoid*)apRobot;
    
    DBG_LOG_INFO("(%s) Gamepad Control Task Started!", "proc_gamepad_control");
    
    RTTIME tmStart = 0, tmCurrent = 0, tmPrev = 0;
    RTTIME tmPeriod = 0, tmMaxPeriod = 0;
    tmStart = read_timer();
    tmPrev = tmStart;
    
    while (!pRobot->CheckStopTask())
    {
        wait_next_period(NULL);
        tmCurrent = read_timer();
        
        // Gamepad 데이터 업데이트 (non-blocking)
        if ((pRobot->GetGamepad() != NULL) && (pRobot->GetGamepad()->IsConnected()))
        {
            pRobot->GetGamepad()->UpdateGamepadData();
            
            bool bLeftTriggerPressed = pRobot->GetGamepad()->GetLeftTrigger() > 0.1;
            
            if (bLeftTriggerPressed) {

                eGamepadButton pressedButton = GAMEPAD_BUTTON_COUNT;
                
                if (pRobot->GetGamepad()->GetButton(GAMEPAD_BUTTON_X))
                    pressedButton = GAMEPAD_BUTTON_X;
                else if (pRobot->GetGamepad()->GetButton(GAMEPAD_BUTTON_A))
                    pressedButton = GAMEPAD_BUTTON_A;
                else if (pRobot->GetGamepad()->GetButton(GAMEPAD_BUTTON_Y))
                    pressedButton = GAMEPAD_BUTTON_Y;
                else if (pRobot->GetGamepad()->GetButton(GAMEPAD_BUTTON_BACK))
                    pressedButton = GAMEPAD_BUTTON_BACK;
                else if (pRobot->GetGamepad()->GetButton(GAMEPAD_BUTTON_START))
                    pressedButton = GAMEPAD_BUTTON_START;
                
                switch (pressedButton)
                    {
                case GAMEPAD_BUTTON_X:
                    DBG_LOG_INFO("[Gamepad] LT+X: Homing Started");
                    pRobot->m_eControlState = HOMING;
                    break;
                        
                case GAMEPAD_BUTTON_A:
                    DBG_LOG_INFO("[Gamepad] LT+A: RL");
                    pRobot->m_eControlState = RL;
                    
                    break;
                case GAMEPAD_BUTTON_Y:
                    DBG_LOG_INFO("[Gamepad] LT+Y: Control Started");
                    pRobot->m_eControlState = CONTROL;
                    break;

                case GAMEPAD_BUTTON_BACK:
                    DBG_LOG_INFO("[Gamepad] LT+Back: INIT Started");
                    pRobot->m_eControlState = INIT;
                    break;

                case GAMEPAD_BUTTON_START:
                    DBG_LOG_INFO("[Gamepad] LT+Start: SetOffset Started");
                    pRobot->m_eControlState = OFFSET;

                    for (int i=0; i < (int)pRobot->GetTotalAxis(); i++) {
                        pRobot->m_pcEcatElmo[i]->UpdateHomeOffset();
                    }   
                    // pRobot->m_pcEcatElmo[4]->UpdateHomeOffset();
                    // pRobot->m_pcEcatElmo[11]->UpdateHomeOffset();
                    break;

                default:
                    // 다른 버튼은 무시
                    break;
                    }
            }
            else {
                // RB 또는 LB 단독 버튼 처리
                if (pRobot->GetGamepad()->GetButton(GAMEPAD_BUTTON_RB) || 
                    pRobot->GetGamepad()->GetButton(GAMEPAD_BUTTON_LB))
                {
                    DBG_LOG_INFO("[Gamepad] RB/LB: Standstill");
                    pRobot->m_eControlState = STANDSTILL;
                }
                
                
            }
        }
        
        // 주기 모니터링 (디버깅용, 선택적)
        // tmPeriod = tmCurrent - tmPrev;
        // if (tmPeriod > tmMaxPeriod) tmMaxPeriod = tmPeriod;
        tmPrev = tmCurrent;
    }
    
    DBG_LOG_WARN("[%s]TASK ENDED!", "proc_gamepad_control");
}

void proc_learning(void* apRobot)
{
    
    CHumanoid* pRobot = (CHumanoid*)apRobot;
    DBG_LOG_INFO("(%s) Learning Task Started!", "proc_learning");

    RTTIME tmStart = 0, tmCurrent=0, tmCurrent2=0;
    tmStart = read_timer();

    Vector<double, 3> vJoyCmd;
    vJoyCmd.setZero();

    while (!pRobot->CheckStopTask())
    {
        wait_next_period(NULL);

        tmCurrent = read_timer();
        // double time_sec = (double)(tmCurrent - tmStart) / 1000000000.0;  // us → sec
        // printf("%lf::Learning Loop\n", time_sec);
        // 관절 위치, 속도, 원하는 위치, IMU 데이터를 RLControl에 전달
        
        if ((pRobot->GetGamepad() != NULL) && (pRobot->GetGamepad()->IsConnected()))
        {
            vJoyCmd(0) = pRobot->GetGamepad()->GetRightStickY()*-1.0*1.0;  // right_x
            vJoyCmd(1) = pRobot->GetGamepad()->GetRightStickX()*1.0*1.0;  // right_y
            vJoyCmd(2) = pRobot->GetGamepad()->GetLeftStickX()*1.0*1.0;   // left_x
        }
        else
        {
            vJoyCmd.setZero();  // 게임패드가 없으면 0으로 설정
        }

        // DBG_LOG_TRACE("[Gamepad] Right Stick: (%.3f, %.3f), Left Stick: (%.3f)", vJoyCmd(0), vJoyCmd(1), vJoyCmd(2));
        
        //RL(@@@@@)
        pRobot->m_pcRLControl->SetRobotState(pRobot->m_pcJointControl->GetJointQrad(), 
                                           pRobot->m_pcJointControl->GetJointQdotrad(), 
                                           pRobot->m_cIMU.GetImuData(),
                                           vJoyCmd);
        if(pRobot->m_eControlState == RL){
            pRobot->m_pcRLControl->Control();
            pRobot->m_pcJointControl->SetDesiredPosition(pRobot->m_pcRLControl->GetDesiredPosition());
        } 

        //mimic(@@@@@)
        // pRobot->m_pcMimicRLControl->SetRobotState(pRobot->m_pcJointControl->GetJointQrad(), 
        //                                    pRobot->m_pcJointControl->GetJointQdotrad(), 
        //                                    pRobot->m_cIMU.GetImuData(),
        //                                    vJoyCmd);
        // if(pRobot->m_eControlState == RL){
        //     if(pRobot->m_pcMimicRLControl->m_bmimic_first_loop == true){
        //         pRobot->m_pcMimicRLControl->m_bmimic_first_loop = false;
        //         pRobot->m_pcMimicRLControl->RunInference();
        //         pRobot->m_pcMimicRLControl->m_vinput_time_step_data[0] += 1.0f;
        //     }
        //     pRobot->m_pcMimicRLControl->Control();
        //     pRobot->m_pcJointControl->SetDesiredPosition(pRobot->m_pcMimicRLControl->GetDesiredPosition());
        // }
    }

    DBG_LOG_WARN("[%s]TASK ENDED!", "proc_learning");
    
}

void
proc_keyboard_control(void* apRobot)
{
	CHumanoid* pRobot = (CHumanoid*)apRobot;

    DBG_LOG_INFO("(%s) Keyboard Input Task Started!", "proc_keyboard_control");
    char cKeyPress = ' ';

    while (!pRobot->CheckStopTask())
    {
        // DBG_LOG_INFO("Keyboard Input Task Running");
        cKeyPress = (char)getche();
        pRobot->m_cKeyPress = cKeyPress;

        if ('q' == cKeyPress)
        {
            pRobot->StopTasks();
            break;
        }
    }
    
    DBG_LOG_WARN("[%s]TASK ENDED!", "proc_keyboard_control");
}

void
proc_terminal_output(void* apRobot)
{
    // printf("Start Terminal\n");
	CHumanoid* pRobot = (CHumanoid*)apRobot;
    (void)pRobot; // temporary: just to avoid warning

    DBG_LOG_INFO("(%s) Terminal Output Task Started!", "proc_terminal_output");

    while (!pRobot->CheckStopTask())
    {
        wait_next_period(NULL);
        

        for (int nMotCnt = 0; nMotCnt < pRobot->GetTotalAxis(); nMotCnt++)                
        {
            
            // DBG_LOG_TRACE("RawVel: %d, Vel: %f", pRobot->m_pcEcatElmo[nMotCnt]->GetCurrentRawVel(), pRobot->m_pcEcatElmo[nMotCnt]->GetCurrentVel());
            // DBG_LOG_TRACE("RawTaretTor: %d, TargetTor: %f, RawTor: %d, CurrentTor: %f", pRobot->m_pcEcatElmo[nMotCnt]->GetTargetRawTor(), 
            //                                                                             pRobot->m_pcEcatElmo[nMotCnt]->GetTargetTorq(),
            //                                                                             pRobot->m_pcEcatElmo[nMotCnt]->GetCurrentRawTor(), 
            //  
            
            // pRobot->m_pcEcatElmo[nMotCnt]->GetCurrentTor();

            if (nMotCnt==23 || nMotCnt==24 || nMotCnt == 30 || nMotCnt==31) {
                DBG_LOG_TRACE("[Slave %d], StatusWord %04x, Mode: %d, IsServoOn: %d, RawInc: %d, IncPos: %lf, RawAbs: %d, AbsPos: %lf, Vel: %lf,RawTarTor: %d, RawTor: %d, TargetTor: %lf, CurrentTor: %lf",
                          nMotCnt,  
                          pRobot->m_pcEcatElmo[nMotCnt]->GetStatusWord(),
                          pRobot->m_pcEcatElmo[nMotCnt]->GetDriveMode(),
                          pRobot->m_pcEcatElmo[nMotCnt]->IsServoOn(),
                          pRobot->m_pcEcatElmo[nMotCnt]->GetCurrentRawPos(),
                          pRobot->m_pcEcatElmo[nMotCnt]->GetCurrentPos(),
                          pRobot->m_pcEcatElmo[nMotCnt]->GetCurrentRawAbsPos(),
                          R2D(pRobot->m_pcEcatElmo[nMotCnt]->GetAdditionalPos()),
                          pRobot->m_pcEcatElmo[nMotCnt]->GetCurrentVel(),
                          pRobot->m_pcEcatElmo[nMotCnt]->GetTargetRawTor(),                          
                          pRobot->m_pcEcatElmo[nMotCnt]->GetCurrentRawTor(),
                          pRobot->m_pcEcatElmo[nMotCnt]->GetTargetTorq(),
                          pRobot->m_pcEcatElmo[nMotCnt]->GetCurrentTor());
            }
            else {
                DBG_LOG_TRACE("[Slave %d], StatusWord %04x, Mode: %d, IsServoOn: %d, RawInc: %d, IncPos: %lf, RawAbs: %d, AbsPos: %lf, Vel: %lf,RawTarTor: %d, RawTor: %d, TargetTor: %lf, CurrentTor: %lf",
                          nMotCnt,  
                          pRobot->m_pcEcatElmo[nMotCnt]->GetStatusWord(),
                          pRobot->m_pcEcatElmo[nMotCnt]->GetDriveMode(),
                          pRobot->m_pcEcatElmo[nMotCnt]->IsServoOn(),
                          pRobot->m_pcEcatElmo[nMotCnt]->GetCurrentRawPos(),
                          R2D(pRobot->m_pcEcatElmo[nMotCnt]->GetCurrentPos()),
                          pRobot->m_pcEcatElmo[nMotCnt]->GetCurrentRawAbsPos(),
                          R2D(pRobot->m_pcEcatElmo[nMotCnt]->GetAdditionalPos()),
                          pRobot->m_pcEcatElmo[nMotCnt]->GetCurrentVel(),
                          pRobot->m_pcEcatElmo[nMotCnt]->GetTargetRawTor(),                          
                          pRobot->m_pcEcatElmo[nMotCnt]->GetCurrentRawTor(),
                          pRobot->m_pcEcatElmo[nMotCnt]->GetTargetTorq(),
                          pRobot->m_pcEcatElmo[nMotCnt]->GetCurrentTor());
            }

            
            
            // DBG_LOG_TRACE("HomePosition: %d StartPos:%f", pRobot->m_pcEcatElmo[nMotCnt]->GetHomePosition(), pRobot->m_pcEcatElmo[nMotCnt]->GetStartRawPos());
            
            // DBG_LOG_TRACE("DriveMode: %s", GetCIA402DriveMode(pRobot->m_pcEcatElmo[nMotCnt]->GetDriveMode()).c_str());
            // DBG_LOG_NOTHING("\n");
                
        }

        
        for (int nHandCnt = 0; nHandCnt < pRobot->GetTotalHands(); nHandCnt++) {
            DBG_LOG_TRACE("Hand[%d] Status1: %d, Status2: %d", nHandCnt, pRobot->m_pcHand[nHandCnt]->GetServoStatus(), pRobot->m_pcHand[nHandCnt]->GetControlMode());
            for (int i = 0; i < pRobot->m_pcHand[nHandCnt]->GetTotalJoints(); i++) {
                DBG_LOG_TRACE("Hand[%d]::J[%d], Pos: %lf, Current: %lf", nHandCnt, i, ConvertRad2Deg(pRobot->m_pcHand[nHandCnt]->GetPosition(i)), ConvertRad2Deg(pRobot->m_pcHand[nHandCnt]->GetCurrent(i)));
            }
            
        }
        
        // Gamepad 상태 출력
        
        if (pRobot->GetGamepad() != NULL && pRobot->GetGamepad()->IsConnected())
        {
            
            // 정규화된 값 가져오기 (deadzone이 이미 적용된 값)
            double leftX = pRobot->GetGamepad()->GetLeftStickX();
            double leftY = pRobot->GetGamepad()->GetLeftStickY();
            double rightX = pRobot->GetGamepad()->GetRightStickX();
            double rightY = pRobot->GetGamepad()->GetRightStickY();
            double leftTrigger = pRobot->GetGamepad()->GetLeftTrigger();
            double rightTrigger = pRobot->GetGamepad()->GetRightTrigger();
            ST_GAMEPAD_STATE state = pRobot->GetGamepad()->GetState();
            // DBG_LOG_TRACE("[Gamepad] LStick: (%.3f, %.3f), RStick: (%.3f, %.3f), LT: %.3f, RT: %.3f, Buttons: %s%s%s%s%s%s%s%s%s%s",
            //                 leftX, leftY, rightX, rightY,
            //                 leftTrigger, rightTrigger,
            //                 state.buttons[GAMEPAD_BUTTON_X] ? "X " : "",
            //                 state.buttons[GAMEPAD_BUTTON_A] ? "A " : "",
            //                 state.buttons[GAMEPAD_BUTTON_B] ? "B " : "",
            //                 state.buttons[GAMEPAD_BUTTON_Y] ? "Y " : "",
            //                 state.buttons[GAMEPAD_BUTTON_LB] ? "LB " : "",
            //                 state.buttons[GAMEPAD_BUTTON_RB] ? "RB " : "",
            //                 state.buttons[GAMEPAD_BUTTON_LEFT_TRIGGER] ? "LT " : "",
            //                 state.buttons[GAMEPAD_BUTTON_RIGHT_TRIGGER] ? "RT " : "",
            //                 state.buttons[GAMEPAD_BUTTON_BACK] ? "Back " : "",
            //                 state.buttons[GAMEPAD_BUTTON_START] ? "Start " : "");
           
        }
        else 
            DBG_LOG_TRACE("[Gamepad] Not connected");
        
        
        
        if (pRobot->m_cIMU.GetImuEnabled())
            DBG_LOG_TRACE("Transformed IMU Euler x:%f, y:%f, z:%f", pRobot->m_cIMU.GetImuData()->euler(0)*57.3, pRobot->m_cIMU.GetImuData()->euler(1)*57.3, pRobot->m_cIMU.GetImuData()->euler(2)*57.3);
        
        // DBG_LOG_TRACE("Transformed IMU Gyro x:%f, y:%f, z:%f", pRobot->m_cIMU.GetImuData()->omega(0), pRobot->m_cIMU.GetImuData()->omega(1), pRobot->m_cIMU.GetImuData()->omega(2));
        // DBG_LOG_TRACE("Transformed IMU Quaternion w:%f, x:%f, y:%f, z:%f\n", pRobot->m_cIMU.GetImuData()->quat(0),
        //                                                                 pRobot->m_cIMU.GetImuData()->quat(1),
        //                                                                 pRobot->m_cIMU.GetImuData()->quat(2),
        //                                                                 pRobot->m_cIMU.GetImuData()->quat(3));

        // DBG_LOG_NOTHING("Lidar Data size: %d, width: %d, height: %d\n", pRobot->m_PointCloudData.data.size(), pRobot->m_PointCloudData.width, pRobot->m_PointCloudData.height);
        // if (!pRobot->m_AxisCmd.positions.empty() && !pRobot->m_AxisCmd.torques.empty()) {
        //     DBG_LOG_NOTHING("Axis Cmd: pos=%f, vel=%f, torq = %f\n", pRobot->m_AxisCmd.positions[0], pRobot->m_AxisCmd.velocities[0], pRobot->m_AxisCmd.torques[0]);
        // }
        
        // DBG_LOG_NOTHING("Axis Cmd: pos=%f, torq = %f\n", pRobot->m_AxisCmd.positions[0], pRobot->m_AxisCmd.torques[0]);
        // DBG_LOG_NOTHING("\n");
    }
    DBG_LOG_WARN("[%s]TASK ENDED!", "proc_terminal_output");
}

void
proc_logger(void* apRobot)
{
    CHumanoid* pRobot = (CHumanoid*)apRobot;
    DBG_LOG_INFO("(%s) Logger Task Started!", "proc_logger_control");

    

    RTTIME tmStart = 0, tmCurrent=0;
    tmStart = read_timer();

    // TSTRING fileName;    
    // sprintf(&fileName[0], "./Log/Log_%ld.csv", tmStart);
    // FILE* pfFile = fopen(fileName.c_str(), "w");
    // if (!pfFile) {
    //     DBG_LOG_ERROR("Failed to open file: %s", fileName.c_str());
    //     return;
    // }

    char buffer[256];
    sprintf(buffer, "./Log/Log_%ld.csv", tmStart);
    TSTRING fileName = buffer;
    
    FILE* pfFile = fopen(fileName.c_str(), "w");
    if (!pfFile) {
        DBG_LOG_ERROR("Failed to open file: %s", fileName.c_str());
        return;
    }


    fprintf(pfFile, "Time");
    PRINT_HEADER_GROUP(1, "KeyCmd");
    PRINT_HEADER_GROUP(pRobot->GetTotalAxis(), "Torque");
    PRINT_HEADER_GROUP(pRobot->GetTotalAxis(), "TorqueDes");
    PRINT_HEADER_GROUP(pRobot->GetTotalAxis(), "MotorPos");
    PRINT_HEADER_GROUP(pRobot->GetTotalAxis(), "MotorDes");
    PRINT_HEADER_GROUP(pRobot->GetTotalAxis(), "Motordot");
    PRINT_HEADER_GROUP(pRobot->GetTotalAxis(), "MotordotDes");
    PRINT_HEADER_GROUP(pRobot->GetTotalAxis(), "MotorAbsPos");
    PRINT_HEADER_GROUP(pRobot->GetTotalAxis(), "Joint");
    PRINT_HEADER_GROUP(pRobot->GetTotalAxis(), "JointDes");
    PRINT_HEADER_GROUP(pRobot->GetTotalAxis(), "Jointdot");
    PRINT_HEADER_GROUP(pRobot->GetTotalAxis(), "JointdotDes");
    PRINT_HEADER_GROUP(pRobot->GetTotalAxis(), "RLTorqueDes");
    PRINT_HEADER_GROUP(pRobot->GetTotalAxis(), "RLJointDes");
    fprintf(pfFile, "\n");


    while (!pRobot->CheckStopTask()) {
        wait_next_period(NULL);

        tmCurrent = read_timer();

        double time_sec = (double)(tmCurrent - tmStart) / 1000000000.0; 

        fprintf(pfFile, "%.9f", time_sec);
        //KeyCmd
        PRINT_GROUP(1, pRobot->m_pcJointControl->m_iControlMode);
        //Torque
        PRINT_GROUP(pRobot->GetTotalAxis(), pRobot->m_pcEcatElmo[i]->GetCurrentTor());
        //TorqueDes
        PRINT_GROUP(pRobot->GetTotalAxis(), pRobot->m_pcJointControl->m_vTorque(i));
        //MotorPos
        PRINT_GROUP(pRobot->GetTotalAxis(), pRobot->m_pcJointControl->m_vMotorThetarad(i));
        //MotorDes
        PRINT_GROUP(pRobot->GetTotalAxis(), pRobot->m_pcJointControl->m_vMotorThetaradDes(i));
        //Motordot
        PRINT_GROUP(pRobot->GetTotalAxis(), pRobot->m_pcJointControl->m_vMotorThetadotrad(i));
        //MotordotDes
        PRINT_GROUP(pRobot->GetTotalAxis(), pRobot->m_pcJointControl->m_vMotorThetadotradDes(i));
        //MotorAbsPos
        PRINT_GROUP(pRobot->GetTotalAxis(), pRobot->m_pcEcatElmo[i]->GetAdditionalPos());
        //Joint
        PRINT_GROUP(pRobot->GetTotalAxis(), pRobot->m_pcJointControl->m_vJointQrad(i));
        //JointDes
        PRINT_GROUP(pRobot->GetTotalAxis(), pRobot->m_pcJointControl->m_vJointQradDes(i));
        //Jointdot
        PRINT_GROUP(pRobot->GetTotalAxis(), pRobot->m_pcJointControl->m_vJointQdotrad(i));
        //JointdotDes
        PRINT_GROUP(pRobot->GetTotalAxis(), pRobot->m_pcJointControl->m_vJointQdotradDes(i));
        //RLTorqueDes
        PRINT_GROUP(pRobot->GetTotalAxis(), pRobot->m_pcJointControl->m_vRLTorque(i));
        //RLJointDes
        PRINT_GROUP(pRobot->GetTotalAxis(), pRobot->m_pcJointControl->m_vRLJointQradDes(i));

        fprintf(pfFile, "\n");

        
        fflush(pfFile);
    
    }

    fclose(pfFile);

    DBG_LOG_WARN("[%s]TASK ENDED!", "proc_logger");

    return;

}

BOOL CHumanoid::PopAxisCmd(axis_info::msg::AxisCmd& cmd)
{
    std::lock_guard<std::mutex> lock(m_mtxAxisCmd);
    if (m_qAxisCmd.empty()) return FALSE;
    
    cmd = m_qAxisCmd.front();
    m_qAxisCmd.pop();
    m_AxisCmd = cmd;
    return TRUE;
}

size_t CHumanoid::GetAxisCmdCount()
{
    std::lock_guard<std::mutex> lock(m_mtxAxisCmd);
    return m_qAxisCmd.size();
}


