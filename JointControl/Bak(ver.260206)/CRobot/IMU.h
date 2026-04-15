#ifndef __IMU__
#define __IMU__

#include <mscl/mscl.h>
#include "commons.h"
#include <Eigen/Dense>
#include <../include/Robotics_func.h>

#define IMU_PORT "/dev/ttyACM0"

struct IMU_ACC {
    double dX = 0.0;
    double dY = 0.0;
    double dZ = 0.0;
};

struct IMU_GYRO {
    double dX = 0.0;
    double dY = 0.0;
    double dZ = 0.0;
};

struct IMU_QUAT {
    double dW = 0.0;
    double dX = 0.0;
    double dY = 0.0;
    double dZ = 0.0;
};


struct IMU_DATA {
    IMU_ACC stAcc{};
    IMU_GYRO stGyro{};
    IMU_QUAT stQuat{};

};


struct IMU_TRANSFORMED_DATA {
    Eigen::Vector3d euler;  // 변환된 오일러 각도 (roll, pitch, yaw)
    Eigen::Vector4d quat;   // 변환된 쿼터니언
    Eigen::Vector3d omega;  // 변환된 각속도
};


const IMU_QUAT IMU_INSTALLATION_POSE = {    // IMU 설치 자세 (똑바로 세웠을때 기준 측정 쿼터니언) : 상수값
    // .dW = 0.3520,    // 현재 측정값
    // .dX = -0.1416,    
    // .dY = 0.9235,
    // .dZ = 0.0564

    .dW = 0.2517,    // 현재 측정값
    .dX = -0.6887,    
    .dY = 0.6182,
    .dZ = 0.2833
};

// 기준 프레임을 상수로 정의 (원하는 자세) - IMU 축 기준 몇도 돌려야 기준프레임이 되는지
// Pitch 90도 + alpha 회전 쿼터니언
const IMU_QUAT DESIRED_FRAME = {
    // (KIM)
    // .dW = _Rot2Quat(_Rot_Z(M_PI) * _Rot_Y(-M_PI / 4.0))(0),
    // .dX = _Rot2Quat(_Rot_Z(M_PI) * _Rot_Y(-M_PI / 4.0))(1),
    // .dY = _Rot2Quat(_Rot_Z(M_PI) * _Rot_Y(-M_PI / 4.0))(2),
    // .dZ = _Rot2Quat(_Rot_Z(M_PI) * _Rot_Y(-M_PI / 4.0))(3)

    // (BAK) yaw 180' for kapex2
    .dW = _Rot2Quat(_Rot_Y(-M_PI / 4.0))(0),
    .dX = _Rot2Quat(_Rot_Y(-M_PI / 4.0))(1),
    .dY = _Rot2Quat(_Rot_Y(-M_PI / 4.0))(2),
    .dZ = _Rot2Quat(_Rot_Y(-M_PI / 4.0))(3)


    // .dW = cos(-M_PI / 8.0),    // cos(-22.5°) = cos(22.5°) ≈ 0.92388
    // .dX = 0.0,                 // x
    // .dY = sin(-M_PI / 8.0),    // sin(-22.5°) = -sin(22.5°) ≈ -0.38268
    // .dZ = 0.0   
};


class CImu
{

public:
    CImu();
    virtual ~CImu();

    BOOL    Connect();
    BOOL    ReadData();
    const   IMU_TRANSFORMED_DATA* GetImuData() const { return &m_stTr; }
    const   IMU_DATA* GetImuDataAhrs() const { return &m_stImuData; }
    const   IMU_DATA* GetImuDataEst() const { return &m_stEstData; }

    BOOL    GetImuEnabled() { return m_bImuEnabled; }
    void    SetImuEnabled(bool bFlag) { m_bImuEnabled = bFlag; }

    std::array<float, 4> m_vQuaternion = {1.f, 0.f, 0.f, 0.f}; // w, x, y, z
    std::array<float, 3> m_vEuler = {0.f, 0.f, 0.f}; // x, y, z
    std::array<float, 3> m_vOmega = {0.f, 0.f, 0.f}; // x, y, z
    

    IMU_TRANSFORMED_DATA transformIMUFrame(const IMU_QUAT& current_quat, const IMU_GYRO& gyro);
    
    

protected:
    mscl::Connection m_Connection;
    mscl::InertialNode *m_pNode;

    IMU_TRANSFORMED_DATA m_stTr;
    IMU_DATA m_stImuData;

    IMU_DATA m_stEstData;

protected:
    void Configure();
    void DumpSupported();

    bool m_bImuEnabled;

};

#endif