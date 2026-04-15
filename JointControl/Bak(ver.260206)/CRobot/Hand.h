#ifndef __KISTAR_HAND__
#define __KISTAR_HAND__

#include "Axis.h"
#include "SlaveKistarSon.h"
#include "EcatMasterBase.h"

#define RAD2USR(X) ((X)*2048.0/M_PI_2)
#define USR2RAD(X) ((X)*M_PI_2/2048.0)

#define NO_COUPLING_JOINTS 5

const INT32 COUPLING_JOINT_INDICES[NO_COUPLING_JOINTS] = {0, 5, 9, 13, 17};

typedef struct stJointLimits
{
	BOOL bIsSet;
	ST_LIMITS stPos;
	ST_LIMITS stCurrent;

	stJointLimits()
	{
		bIsSet = FALSE;
	}

}ST_JOINT_LIMITS;

typedef struct stRawHandJointInfo {
    INT16 nPos;
    INT16 nCurrent;

    stRawHandJointInfo() {
        nPos = 0;
        nCurrent = 0;
    }
} ST_RAW_HAND_JOINT_INFO;

typedef struct stHandJointInfo {
    double dPos;
    double dCurrent;

    stHandJointInfo() {
        dPos = 0.0;
        dCurrent = 0.0;
    }
} ST_HAND_JOINT_INFO;



typedef enum {
    eHAND_VOLTAGE = 0,
    eHAND_POSITION = 1,
    eHAND_IMPEDANCE,
    eHAND_CURRENT,
} EControlMode;


class CHand
{
public:
    CHand(INT32 nNoAxes, INT32 nNoJoints, BOOL abEnabled, BOOL abConnected);
    virtual ~CHand();

public:
    BOOL Init(CEcatMaster&);
    BOOL isConnected() { return m_bConnected; }

    void SetEnabled(BOOL bEnabled) { m_bEnabled = bEnabled; }
    void SetServoOn(UINT16 usFlag=0xFFFF) { m_cEcSlave.SetServoStatus(usFlag); }
    void SetServoOff(UINT16 usFlag=0x0000) {m_cEcSlave.SetServoStatus(usFlag); }
    void SetControlMode(EControlMode eMode) { m_cEcSlave.SetControlMode(eMode); }
    

    INT16 GetNoAxes() { return m_nNoAxes; }    
    double GetTargetPos(int nJoint);
    INT16 GetRawPosition(int nJoint);
    double GetPosition(int nJoint);
    INT16 GetRawCurrent(int nJoint);
    double GetCurrent(int nJoint);
    INT16 GetKinesthetic(int nNo) { return m_cEcSlave.GetKinesthetic(nNo); }
    INT16 GetTactile(int nNo) { return m_cEcSlave.GetTactile(nNo); }
    // INT16 GetAddInfoIn(int nNo) { return m_cEcSlave.GetAddInfoIn(nNo); }
    INT16 GetServoStatus() { return m_cEcSlave.GetServoStatus(); }
    INT16 GetControlMode() { return m_cEcSlave.GetControlMode(); }
    INT16 GetStatusOut1() { return m_cEcSlave.GetStatusOut1(); }
    INT16 GetStatusOut2() { return m_cEcSlave.GetStatusOut2(); }

    void SetMasterID(INT32 nMasterID) { m_nMasterID = nMasterID; }
    INT32 GetMasterID() { return m_nMasterID; }
    void SetName(TSTRING astrName) { m_strName = astrName; }
	TSTRING	GetName() { return m_strName; }
    void SetVendorInfo(UINT32 auVendorID, UINT32 auProductCode) { m_cEcSlave.SetVendorInfo(auVendorID, auProductCode); };
    void SetDCInfo(BOOL abDCSupported, UINT16 ausActivateWord, INT32 anShiftTime) { m_cEcSlave.SetDCInfo(abDCSupported, ausActivateWord, anShiftTime); };
    void SetPositionLimits(INT32 nJoint, double dPosLimitL, double dPosLimitU, BOOL abSet = TRUE);    
    ST_LIMITS GetPositionLimits(INT32 nJoint) { return m_pstJointLimits[nJoint].stPos; }
    INT32 GetTotalJoints(void) { return m_nTotalJoints; }
    
    BOOL MovePosition(int nJoint, double dTargetPos);
    void UpdateJointInfo();
    

private:
    BOOL m_bEnabled;
    TSTRING	m_strName;

    ST_JOINT_LIMITS* m_pstJointLimits;
    ST_HAND_JOINT_INFO* m_pstHandJointInfo;
    ST_RAW_HAND_JOINT_INFO* m_pstRawHandJointInfo;

    INT32 GetAxisIndex(INT32 nJointIndex);



protected:
	CSlaveKistarSon	    m_cEcSlave;
	CEcatMaster*	    m_pEcMaster;
    BOOL                m_bConnected;
    INT32               m_nMasterID;

    INT16               m_nNoAxes;
    INT8                m_nTotalJoints;

    virtual void	OnSlaveUpdateRawParam	(PVOID, PVOID, PVOID, PVOID);
    BOOL IsCouplingJoint(INT32 nIndex);
    BOOL SetTargetPos(int nJoint, double dTarget);
    BOOL IsAllowablePosition(int nJoint, double dPos);
    BOOL CheckLimits(double dTarget, ST_LIMITS stLimit);
    
};


#endif