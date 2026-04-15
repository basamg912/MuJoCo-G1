#include "IMU.h"
#include "Defines.h"

CImu::CImu()
{


    SetImuEnabled(TRUE);

    if (m_bImuEnabled) {
        m_Connection = mscl::Connection::Serial(IMU_PORT, 921600);
        m_pNode = new mscl::InertialNode(m_Connection);
    }

    
    
}

CImu::~CImu()
{
    try {
        m_pNode->setToIdle();
    } catch (...) {

    }
    delete m_pNode;

    m_Connection.clearBuffer();
    m_Connection.disconnect();

}

BOOL CImu::Connect()
{
    
    for (int i=0; i<5; ++i) {
        try {
            if (m_pNode->ping()) {
                m_pNode->setToIdle();
                // DumpSupported();
                Configure();
                DBG_INFO("[%s] Connected to IMU", "IMU");
                return TRUE;
            }
        } catch (const mscl::Error_Connection& e) {
            DBG_ERROR("[%s] Connection Error::%s", "IMU", e.what());
            
        } catch (const mscl::Error& e) {
            DBG_ERROR("[%s] MSCL Error::%s", "IMU", e.what());
        }
        usleep(100);
    }
    
    DBG_ERROR("[%s] Fail to Connect IMU", "IMU");
    return FALSE;
}

void CImu::DumpSupported()
{
    auto &features = m_pNode->features();

    std::cout << "=== Supported AHRS/IMU channel fields ===" << std::endl;
    auto fields = features.supportedChannelFields(mscl::MipTypes::CLASS_AHRS_IMU);
    for(auto f : fields)
    {
        std::cout << "  field id = " << static_cast<int>(f) << std::endl;
    }

    std::cout << "=== Supported AHRS/IMU sample rates ===" << std::endl;
    auto rates = features.supportedSampleRates(mscl::MipTypes::CLASS_AHRS_IMU);
    for(const auto& r : rates)
    {
        std::cout << "  " << r.prettyStr() << std::endl;
    }
}

void CImu::Configure()
{
    mscl::MipChannels chs_ahrs;
    mscl::MipChannels chs_est;
    auto sr = mscl::SampleRate::Hertz(1000);

    // chs.push_back(mscl::MipChannel(mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_RAW_ACCEL_VEC, sr));
    // chs.push_back(mscl::MipChannel(mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_RAW_GYRO_VEC, sr));
    // chs.push_back(mscl::MipChannel(mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_RAW_MAG_VEC, sr));
    chs_ahrs.push_back(mscl::MipChannel(mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_ORIENTATION_QUATERNION, sr));
    chs_ahrs.push_back(mscl::MipChannel(mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_ACCEL_VEC,       sr));
    chs_ahrs.push_back(mscl::MipChannel(mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_GYRO_VEC,        sr));

    // chs_est.push_back(mscl::MipChannel(mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_LINEAR_ACCEL, sr));
    // chs_est.push_back(mscl::MipChannel(mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION,       sr));
    // chs_est.push_back(mscl::MipChannel(mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE,        sr));
    
    

    try
    {
        m_pNode->setActiveChannelFields(mscl::MipTypes::DataClass::CLASS_AHRS_IMU, chs_ahrs);
        // m_pNode->setActiveChannelFields(mscl::MipTypes::DataClass::CLASS_ESTFILTER, chs_est);
        m_pNode->enableDataStream(mscl::MipTypes::DataClass::CLASS_AHRS_IMU, true);
        // m_pNode->enableDataStream(mscl::MipTypes::CLASS_ESTFILTER, true);
    }
    catch(const mscl::Error& e)
    {
        std::cerr << "[IMU] setActiveChannelFields error: " << e.what() << std::endl;
    }

    
}

 IMU_TRANSFORMED_DATA CImu::transformIMUFrame(const IMU_QUAT& current_quat, const IMU_GYRO& gyro) {
    IMU_TRANSFORMED_DATA result;

    // 현재 쿼터니언을 회전행렬로 변환 {IMU}
    Eigen::Vector4d current_vec(current_quat.dW, current_quat.dX, current_quat.dY, current_quat.dZ);
    Eigen::Matrix3d R_current_raw = _Quat2Rot(current_vec);

    // IMU 설치 자세를 회전행렬로 변환 {IMU} (생성자에서 한번만 호출하면 됨.)
    Eigen::Vector4d install_vec(IMU_INSTALLATION_POSE.dW, IMU_INSTALLATION_POSE.dX, IMU_INSTALLATION_POSE.dY, IMU_INSTALLATION_POSE.dZ);
    Eigen::Matrix3d R_install_raw = _Quat2Rot(install_vec);
    
    // Desired frame을 회전행렬로 변환 {I} -> {IMU} (생성자에서 한번만 호출하면 됨.)
    Eigen::Vector4d desired_vec(DESIRED_FRAME.dW, DESIRED_FRAME.dX, DESIRED_FRAME.dY, DESIRED_FRAME.dZ);
    Eigen::Matrix3d R_desired = _Quat2Rot(desired_vec).transpose();

    Eigen::Matrix3d R_corrected = R_desired * (R_install_raw.transpose() * R_current_raw) * R_desired.transpose();

    Eigen::Vector3d gyro_vec(gyro.dX, gyro.dY, gyro.dZ);

    static double init_yaw_angle = 0.0;
    static bool isInit = true;

    if (isInit) {        
        // Quaternion으로 초기 yaw 계산 (Gimbal Lock 방지)
        Eigen::Vector4d corrected_quat = _Rot2Quat(R_corrected);
        init_yaw_angle = 2.0 * atan2(corrected_quat(3), corrected_quat(0));  // Yaw from quaternion
        
        isInit = false;

        // Quaternion으로 최종 변환
        Eigen::Matrix3d R_z_initial = _Rot_Z(-init_yaw_angle);
        Eigen::Matrix3d R_final = R_z_initial * R_corrected;
           
        // Quaternion으로 결과 반환 (Gimbal Lock 방지)
        result.quat = _Rot2Quat(R_final);
           
           // Euler Angle은 참고용으로만 (Gimbal Lock 주의)
        result.euler = _Rot2Eul(R_final);
    } else {

        // Quaternion으로 최종 변환
        Eigen::Matrix3d R_z_initial = _Rot_Z(-init_yaw_angle);
        Eigen::Matrix3d R_final = R_z_initial * R_corrected;
        
        // Quaternion으로 결과 반환 (Gimbal Lock 방지)
        result.quat = _Rot2Quat(R_final);
        
        // Euler Angle은 참고용으로만 (Gimbal Lock 주의)
        result.euler = _Rot2Eul(R_final);
    }
    
    // 각속도 Frame 변환
    result.omega = R_desired * gyro_vec;
    
    return result;
}


BOOL CImu::ReadData()
{
    mscl::MipDataPackets Latest;
    while (TRUE) {
        mscl::MipDataPackets tmp = m_pNode->getDataPackets(0, 0);
        if (tmp.empty()) break;
        Latest = std::move(tmp);
    }
    if (Latest.empty()) return FALSE;

    const mscl::MipDataPoints& pts = Latest.back().data();

    bool bAccX=FALSE, bAccY=FALSE, bAccZ=FALSE;
    bool bGyrX=FALSE, bGyrY=FALSE, bGyrZ=FALSE;
    bool bQuat=FALSE;

    

    for (const auto& pt : pts) {
        const std::string ch = pt.channelName();

        // printf("%s\n", ch.c_str());

        
        if      (ch == "scaledAccelX") { m_stImuData.stAcc.dX = pt.as_double(); bAccX = TRUE; }
        else if (ch == "scaledAccelY") { m_stImuData.stAcc.dY = pt.as_double(); bAccY = TRUE; }
        else if (ch == "scaledAccelZ") { m_stImuData.stAcc.dZ= pt.as_double(); bAccZ = TRUE; }
        
        else if (ch == "scaledGyroX")  { m_stImuData.stGyro.dX = pt.as_double(); bGyrX = TRUE; }
        else if (ch == "scaledGyroY")  { m_stImuData.stGyro.dY = pt.as_double(); bGyrY = TRUE; }
        else if (ch == "scaledGyroZ")  { m_stImuData.stGyro.dZ = pt.as_double(); bGyrZ = TRUE; }
        
        else if (ch == "orientQuaternion") {
            try {
                mscl::Vector qv = pt.as_Vector();
                const size_t n = qv.size();
                if (n >= 4) {                    
                    m_stImuData.stQuat.dW = static_cast<double>(qv.as_floatAt(0));
                    m_stImuData.stQuat.dX = static_cast<double>(qv.as_floatAt(1));
                    m_stImuData.stQuat.dY = static_cast<double>(qv.as_floatAt(2));
                    m_stImuData.stQuat.dZ = static_cast<double>(qv.as_floatAt(3));
                    bQuat = true;

                    m_stTr = transformIMUFrame(m_stImuData.stQuat, m_stImuData.stGyro);

                    for(int i=0; i<4; i++){
                        m_vQuaternion[i] = m_stTr.quat(i);
                    }
                    for(int i=0; i<3; i++){
                        m_vEuler[i] = m_stTr.euler(i);
                        m_vOmega[i] = m_stTr.omega(i);
                    }

                    // printf("raw quat: [%.4f, %.4f, %.4f, %.4f]\n", 
                    //                    m_stImuData.stQuat.dW, m_stImuData.stQuat.dX, m_stImuData.stQuat.dY, m_stImuData.stQuat.dZ);
  

                    // printf("Roll: %.3f°, Pitch: %.3f°, Yaw: %.3f°\n", 
                    //                    R2D(m_stTr.euler(0)), R2D(m_stTr.euler(1)), R2D(m_stTr.euler(2)));
                    // printf("Quat: [%.4f, %.4f, %.4f, %.4f]\n", 
                    //                    m_stTr.quat(0), m_stTr.quat(1), m_stTr.quat(2), m_stTr.quat(3));
                    // printf("Omega: [%.4f, %.4f, %.4f]\n", 
                    //                    m_stTr.omega(0), m_stTr.omega(1), m_stTr.omega(2));

                }
            
            } catch (...) {
                
            }
        }
        else if (ch == "estAngularRateX") {
            m_stEstData.stGyro.dX = pt.as_double();
        }
        else if (ch == "estAngularRateY") {
            m_stEstData.stGyro.dY = pt.as_double();
        }
        else if (ch == "estAngularRateZ") {
            m_stEstData.stGyro.dZ = pt.as_double();
        }
        else if (ch == "estLinearAccelX") {
            m_stEstData.stAcc.dX = pt.as_double();
        }
        else if (ch == "estLinearAccelY") {
            m_stEstData.stAcc.dY = pt.as_double();
        }
        else if (ch == "estLinearAccelZ") {
            m_stEstData.stAcc.dZ = pt.as_double();
        }
        else if (ch == "estOrientQuaternion") {
            mscl::Vector qv = pt.as_Vector();
            const size_t n = qv.size();
            if (n >= 4) {                    
                m_stEstData.stQuat.dW = static_cast<double>(qv.as_floatAt(0));
                m_stEstData.stQuat.dX = static_cast<double>(qv.as_floatAt(1));
                m_stEstData.stQuat.dY = static_cast<double>(qv.as_floatAt(2));
                m_stEstData.stQuat.dZ = static_cast<double>(qv.as_floatAt(3));
            }
        }
        
    }

    return TRUE;
}