/*****************************************************************************
*	Name: ConfigRobot.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for CConfigRobot class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef __CONFIG_ROBOT__
#define __CONFIG_ROBOT__

#include "Defines.h"
#include "ConfigParser.h"

#define DEFAULT_CONFIGURATION_FULLPATH ("../Config/KAPEX_all.cfg")

typedef struct stConfigTask
{
	BOOL	bEnabled;
	TSTRING	strName;
	INT32	nPriority;
	INT32	nStartDelay;
	INT32	nPeriod;

	stConfigTask()
	{
		bEnabled = FALSE;
		strName = "";
		nPriority = 0;
		nStartDelay = 0;
		nPeriod = 0;
	}
}ST_CONFIG_TASK;

typedef struct stConfigEcatMaster
{
	BOOL	bDCEnabled;
	INT32	nNoOfSlaves;
	INT32 	nNoOfHands;
	INT32	nCycleTime;

	stConfigEcatMaster()
	{
		bDCEnabled = FALSE;
		nNoOfSlaves = 0;
		nNoOfHands = 0;
		nCycleTime = 0;
	}

}ST_CONFIG_ECAT_MASTER;

typedef struct stConfigEcatSlave
{
	BOOL	bEnabled;
	BOOL	bConnected;
	TSTRING	strName;
	INT32	nPhyPos;
	UINT32	uVendorID;
	UINT32	uProductCode;
	BOOL	bDCSupported;
	UINT16	usDCActivateWord;
	INT32	nDCSyncShift;

	BOOL	bIsQuartet;
	INT32	nQuartetID;		// Group ID for quartet (multiple quartets can exist)
	INT32	nQuartetIdx;	// Axis index within quartet (0,1,2,3)
	INT32	nMasterID;

	BOOL	bAbsEnc;
	BOOL	bCCW;
	INT32	nJointType;
	INT32	nDriveMode;
	double	dEncResolution;
	double 	dAbsEncResolution;
	double	dGearRatio;
	double	dTransRatio;
	double	dOneTurnRef;
	
	INT32	nHomingMethod;
	double	dHomeSearchRef;
	INT32	nHomePositionOffset;

	double	dOperatingVel;
	double	dOperatingAcc;
	double	dOperatingDec;

	double	dPosLimitU;
	double	dPosLimitL;

	double	dVelLimitU;
	double	dVelLimitL;

	double	dAccLimitU;
	double	dAccLimitL;

	double	dDecLimitU;
	double	dDecLimitL;

	double dTorLimitU;
	double dTorLimitL;

	double	dJerkLimitU;
	double	dJerkLimitL;

	double dRatedTorque;
	double dRatedCurrent;

	double dPGain;
	double dDGain;

	INT32	nPosBeforeExit;

	stConfigEcatSlave()
	{
		bEnabled = FALSE;
		bConnected = FALSE;
		strName = "EtherCAT Slave";
		nPhyPos = 0;
		uVendorID = 0;
		uProductCode = 0;
		bDCSupported = FALSE;
		usDCActivateWord = 0x300;
		nDCSyncShift = 0;
		bIsQuartet = FALSE;
		nQuartetID = 0;
		nQuartetIdx = 0;
		nMasterID = 0;

		bAbsEnc = FALSE;
		bCCW = TRUE;
		nJointType = 0;
		nDriveMode = 1;
		dEncResolution = 1.;
		dAbsEncResolution = 1.0;
		dGearRatio = 1.;
		dTransRatio = 1.;
		dOneTurnRef = 1.;

		nHomingMethod = 0;
		dHomeSearchRef = 1.;
		nHomePositionOffset = 0;

		dOperatingVel = 0.;
		dOperatingAcc = 0.;
		dOperatingDec = 0.;

		dPosLimitL = 0.;
		dPosLimitU = 0.;

		dVelLimitL = 0.;
		dVelLimitU = 0.;

		dAccLimitL = 0.;
		dAccLimitU = 0.;

		dDecLimitL = 0.;
		dDecLimitU = 0.;
		
		dTorLimitL = 0.;
		dTorLimitU = 0.;
		
		dJerkLimitL = 0.;
		dJerkLimitU = 0.;

		dRatedTorque = 0.0;
		dRatedCurrent = 0.0;

		dPGain = 0.0;
		dDGain = 0.0;

		nPosBeforeExit = 0;
	}
}ST_CONFIG_ECAT_SLAVE;


typedef struct stHandConfigEcatSlave {
	BOOL	bEnabled;
	BOOL	bConnected;
	TSTRING	strName;
	INT32	nPhyPos;
	UINT32	uVendorID;
	UINT32	uProductCode;
	INT32	nMasterID;
	BOOL	bDCSupported;
	UINT16	usDCActivateWord;
	INT32	nDCSyncShift;
	INT32 	nNoAxes;
	INT32 	nNoJoints;

	double dPosLimitU[20];
	double dPosLimitL[20];
	
	stHandConfigEcatSlave()
	{
		bEnabled = FALSE;
		bConnected = FALSE;
		strName = "EtherCAT Hand";
		nPhyPos = 0;
		uVendorID = 0;
		uProductCode = 0;
		nMasterID = 0;
		bDCSupported = FALSE;
		usDCActivateWord = 0x300;
		nDCSyncShift = 0;
		nNoAxes = 0;
		nNoJoints = 0;
		
		for (int i = 0; i < 20; i++)
		{
			dPosLimitU[i] = 0.;
			dPosLimitL[i] = 0.;
		}
	}

} ST_HAND_CONFIG_ECAT_SLAVE;




typedef struct stConfigExtInterface
{
	BOOL	bEnabled;
	INT32	nPort;

	stConfigExtInterface()
	{
		bEnabled = FALSE;
		nPort = 7420;
	}
}ST_CONFIG_EXT_IFACE;

typedef struct stConfigSystem
{
	TSTRING strName;
	BOOL	bSim;
	BOOL	bAging;
	INT32	nAgingDur;
	INT32	nTeleMode;
	double	dTeleScalingRatio;

	ST_CONFIG_EXT_IFACE stExternalIface;

	stConfigSystem()
	{
		strName = "ROBOT";
		bSim = FALSE;
		bAging = FALSE;
		nTeleMode = 0;
		dTeleScalingRatio = 0.;
	};

}ST_CONFIG_SYSTEM;

typedef std::vector<ST_CONFIG_TASK>			VEC_TASK_CONF;
typedef std::vector<ST_CONFIG_ECAT_SLAVE>	VEC_ECAT_SLAVE_CONF;
typedef std::vector<ST_HAND_CONFIG_ECAT_SLAVE>	VEC_HAND_ECAT_SLAVE_CONF;

class CConfigRobot
{
public:
	CConfigRobot();
	virtual ~CConfigRobot();

public:
	void						SetConfigPath		(TSTRING astrPath)	{ m_strConfigPath = astrPath; }
	TSTRING						GetConfigPath		(	)				{ return m_strConfigPath; }
	BOOL						ReadConfiguration	(TSTRING astrPath = "DEFAULT");
	void						LoadDefaultConfig	(	);
	BOOL						UpdateConfiguration	(	);
	BOOL						CreateDefaultConfig	(	);

	VEC_TASK_CONF				GetTaskList			(	) { return m_vstTaskList; }
	VEC_ECAT_SLAVE_CONF			GetEcatSlaveList	(	) { return m_vstEcatSlaveList; }
	VEC_HAND_ECAT_SLAVE_CONF	GetHandEcatSlaveList(	) { return m_vstHandEcatSlaveList; }
	ST_CONFIG_ECAT_MASTER		GetEcatMasterConf	(	) { return m_stEcatMaster; }
	ST_CONFIG_SYSTEM			GetSystemConf		(	) { return m_stSystem; }
	ST_CONFIG_EXT_IFACE			GetExtIfaceConf		(	) { return m_stSystem.stExternalIface; }
	void						WriteLastPosition	(INT32, INT32);

private:
	BOOL	ReadRTTaskConfig	(	);
	BOOL	ReadEtherCATConfig	(	);
	BOOL	ReadSystemConfig	(	);

private:
	TSTRING					m_strConfigPath;
	ST_CONFIG_ECAT_MASTER	m_stEcatMaster;
	ST_CONFIG_SYSTEM		m_stSystem;
	ST_CONFIG_EXT_IFACE		m_stExtIface;
	VEC_TASK_CONF			m_vstTaskList;
	VEC_ECAT_SLAVE_CONF		m_vstEcatSlaveList;
	VEC_HAND_ECAT_SLAVE_CONF m_vstHandEcatSlaveList;

}; // CConfigRobot



#endif // __CONFIG_ROBOT__


