/*****************************************************************************
*	Name: ConfigRobot.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implementation of the CConfigRobot class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include "ConfigRobot.h"

CConfigRobot::CConfigRobot()
{
	m_vstTaskList.clear();
	m_vstEcatSlaveList.clear();
	
	m_vstHandEcatSlaveList.clear();
}

CConfigRobot::~CConfigRobot()
{

}

BOOL
CConfigRobot::ReadConfiguration(TSTRING astrPath)
{
	try
	{
		m_vstTaskList.clear();
		m_vstEcatSlaveList.clear();
		
		m_vstHandEcatSlaveList.clear();

		TSTRING strPath = m_strConfigPath;
		if ("DEFAULT" != astrPath)
			strPath = astrPath;

		if (!IsExistFile(strPath.c_str()))
		{
			DBG_LOG_ERROR("(%s) Cannot read configuration file %s!", "CConfigRobot", strPath.c_str());
			return FALSE;
		}
		SetConfigPath(strPath);

		if (FALSE == ReadSystemConfig())
		{
			DBG_LOG_ERROR("(%s) Cannot read System Configuration!", "CConfigRobot");
			return FALSE;
		}
		
		if (FALSE == ReadRTTaskConfig())
		{
			DBG_LOG_ERROR("(%s) Cannot read RT-TASK Configuration!", "CConfigRobot");
			return FALSE;
		}

		if (FALSE == ReadEtherCATConfig())
		{
			DBG_LOG_ERROR("(%s) Cannot read EtherCAT Configuration!", "CConfigRobot");
			return FALSE;
		}

	}
	catch (...)
	{
		DBG_LOG_ERROR("(%s) ReadConfiguration -  Exception Occurs!", "CConfigRobot");
		return FALSE;
	}

	return TRUE;
}

BOOL
CConfigRobot::ReadRTTaskConfig()
{
	try
	{
		TSTRING strKey = "";
		TCHAR	str[__MAX_PATH__];

		strKey = "RT_TASKS";
		ZeroMemory(str);
		GetPrivateProfileString(strKey.c_str(), "NO_OF_TASKS", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
		INT nNoOfTasks = atoi(str);

		/* iterate on all tasks */
		for (int nCnt = 0; nCnt < nNoOfTasks; nCnt++)
		{
			ST_CONFIG_TASK stTask;
			strKey = "TASK" + TOSTRING(nCnt);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "ENABLED", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stTask.bEnabled = (BOOL)atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "NAME", "NULL", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			if (strlen(str) > 0 && strlen(str) < __MAX_PATH__)
				stTask.strName = TSTRING(str);
			else
				stTask.strName = "NULL";

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "PRIORITY", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stTask.nPriority = atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "TASK_PERIOD", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stTask.nPeriod = atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "START_DELAY", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stTask.nStartDelay = atoi(str);

			m_vstTaskList.push_back(stTask);
		}
	}
	catch (...)
	{
		DBG_LOG_ERROR("(%s) ReadRTTaskConfig - Exception Occurs!", "CConfigRobot");
		return FALSE;
	}

	return TRUE;
}

BOOL
CConfigRobot::ReadEtherCATConfig()
{
	try
	{
		TSTRING strKey = "";
		TCHAR	str[__MAX_PATH__];

		

	
		strKey = "ECAT_MASTER";
		ZeroMemory(str);
		GetPrivateProfileString(strKey.c_str(), "SLAVENUMBER", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
		m_stEcatMaster.nNoOfSlaves = atoi(str);

		ZeroMemory(str);
		GetPrivateProfileString(strKey.c_str(), "SLAVEHANDNUMBER", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
		m_stEcatMaster.nNoOfHands = atoi(str);

		ZeroMemory(str);
		GetPrivateProfileString(strKey.c_str(), "DC_ENABLED", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
		m_stEcatMaster.bDCEnabled = (BOOL)atoi(str);

		ZeroMemory(str);
		GetPrivateProfileString(strKey.c_str(), "CYCLE_TIME", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
		m_stEcatMaster.nCycleTime = atoi(str);
		
		
		int nJunctionCnt = 0;
		int nElmoCnt = 0;
		
		/* iterate on all slaves */
		INT16 nAdd4Axis = 6;
		
		for (int nCnt = 0; nCnt < (m_stEcatMaster.nNoOfSlaves+nAdd4Axis); nCnt++)
		{
			ST_CONFIG_ECAT_SLAVE stSlave;

			
			if ((nCnt == 0) || (nCnt == 15)) {			
				strKey = "JUNCTION" + TOSTRING(nJunctionCnt);
				nJunctionCnt++;				
			}			
			else {
				
				strKey = "AXIS" + TOSTRING(nElmoCnt);
				nElmoCnt++;
			}
				
			

			// strKey = "AXIS" + TOSTRING(nCnt);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "ENABLED", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.bEnabled = (BOOL)atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "CONNECTED", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.bConnected = (BOOL)atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "NAME", "EtherCAT Slave", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			if (strlen(str) > 0 && strlen(str) < __MAX_PATH__)
				stSlave.strName = TSTRING(str);
			else
				stSlave.strName = "EtherCAT Slave";

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "PHYSICAL_POS", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.nPhyPos = atoi(str);

			/**/
			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "VENDOR_ID", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			try 
			{
				stSlave.uVendorID = (UINT32)std::stoul(str, NULL, 16); //hex to uint32
			}
			catch (std::invalid_argument&)
			{
				stSlave.uVendorID = (UINT32)atoi(str); 
			}
			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "PRODUCT_CODE", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			try
			{
				stSlave.uProductCode = (UINT32)std::stoul(str, NULL, 16); //hex to uint32
			}
			catch (std::invalid_argument&)
			{
				stSlave.uProductCode = (UINT32)atoi(str); 
			}

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "DC_SUPPORT", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.bDCSupported = (BOOL)atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "DC_ACTIVATE_WORD", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			try
			{
				stSlave.usDCActivateWord = (UINT16)std::stoul(str, NULL, 16); //hex to uint16
			}
			catch (std::invalid_argument&)
			{
				stSlave.usDCActivateWord = (UINT16)atoi(str); 
			}

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "DC_SYNC0_SHIFT", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.nDCSyncShift = atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "IS_QUARTET", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.bIsQuartet = (BOOL)atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "QUARTET_ID", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.nQuartetID = atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "QUARTET_IDX", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.nQuartetIdx = atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "MASTER_ID", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.nMasterID = atoi(str);
		
		
			/**/

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "ABSOLUTE_ENCODER", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.bAbsEnc = (BOOL)atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "MOVE_CCW", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.bCCW = (BOOL)atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "JOINT_TYPE", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.nJointType = atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "DRIVE_MODE", "1", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.nDriveMode = atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "ENCODER_RESOLUTION", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dEncResolution = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "ABS_ENCODER_RESOLUTION", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dAbsEncResolution = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "GEAR_RATIO", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dGearRatio = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "TRANSMISSION_RATIO", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dTransRatio = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "ONE_TURN_REF", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dOneTurnRef = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "HOMING_METHOD", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.nHomingMethod = atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "HOME_POSITION_OFFSET", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.nHomePositionOffset = atoi(str);
			
			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "HOME_SEARCH_REFERENCE", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dHomeSearchRef = atof(str);


			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "OPERATING_VELOCITY", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dOperatingVel = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "OPERATING_ACCELERATION", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dOperatingAcc = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "OPERATING_DECELERATION", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dOperatingDec = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "POSITION_LIMIT_L", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dPosLimitL = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "POSITION_LIMIT_U", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dPosLimitU = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "VELOCITY_LIMIT_L", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dVelLimitL = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "VELOCITY_LIMIT_U", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dVelLimitU = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "ACCELERATION_LIMIT_L", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dAccLimitL = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "ACCELERATION_LIMIT_U", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dAccLimitU = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "DECELERATION_LIMIT_L", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dDecLimitL = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "DECELERATION_LIMIT_U", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dDecLimitU = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "TORQUE_LIMIT_L", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dTorLimitL = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "TORQUE_LIMIT_U", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dTorLimitU = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "JERK_LIMIT_L", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dJerkLimitL = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "JERK_LIMIT_U", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dJerkLimitU = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "RATED_TORQUE", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dRatedTorque = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "RATED_CURRENT", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dRatedCurrent = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "P_GAIN", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dPGain = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "D_GAIN", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.dDGain = atof(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "POS_BEFORE_EXIT", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.nPosBeforeExit = atoi(str);

			m_vstEcatSlaveList.push_back(stSlave);
		}

		for (int nCnt = 0; nCnt < m_stEcatMaster.nNoOfHands; nCnt++) {
			
			ST_HAND_CONFIG_ECAT_SLAVE stSlave;
			strKey = "HAND" + TOSTRING(nCnt);
			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "ENABLED", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.bEnabled = (BOOL)atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "CONNECTED", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.bConnected = (BOOL)atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "NAME", "EtherCAT Slave", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			if (strlen(str) > 0 && strlen(str) < __MAX_PATH__)
				stSlave.strName = TSTRING(str);
			else
				stSlave.strName = "EtherCAT Slave";

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "PHYSICAL_POS", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.nPhyPos = atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "VENDOR_ID", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			try 
			{
				stSlave.uVendorID = (UINT32)std::stoul(str, NULL, 16); //hex to uint32
			}
			catch (std::invalid_argument&)
			{
				stSlave.uVendorID = (UINT32)atoi(str); 
			}
			
			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "PRODUCT_CODE", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			try
			{
				stSlave.uProductCode = (UINT32)std::stoul(str, NULL, 16); //hex to uint32
			}
			catch (std::invalid_argument&)
			{
				stSlave.uProductCode = (UINT32)atoi(str); 
			}

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "MASTER_ID", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.nMasterID = atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "NO_AXES", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.nNoAxes = atoi(str);

			ZeroMemory(str);
			GetPrivateProfileString(strKey.c_str(), "NO_JOINTS", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
			stSlave.nNoJoints = atoi(str);

			for (int j = 0; j < stSlave.nNoJoints; j++) {
				ZeroMemory(str);
				GetPrivateProfileString(strKey.c_str(), ("POSITION_LIMIT_L"+TOSTRING(j)).c_str(), "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
				stSlave.dPosLimitL[j] = atof(str);


				ZeroMemory(str);
				GetPrivateProfileString(strKey.c_str(), ("POSITION_LIMIT_U"+TOSTRING(j)).c_str(), "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
				stSlave.dPosLimitU[j] = atof(str);

			}				

			m_vstHandEcatSlaveList.push_back(stSlave);
		
			
		}
		
		
	}
	catch (...)
	{
		DBG_LOG_ERROR("(%s) ReadEtherCATConfig - Exception Occurs!", "CConfigRobot");
		return FALSE;
	}

	return TRUE;
}

BOOL
CConfigRobot::ReadSystemConfig()
{
	try
	{
		TSTRING strKey = "";
		TCHAR	str[__MAX_PATH__];

		strKey = "SYSTEM";
		ZeroMemory(str);
		GetPrivateProfileString(strKey.c_str(), "SIMULATION_MODE", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
		m_stSystem.bSim = (BOOL)atoi(str);

		
		/* Aging test */
		ZeroMemory(str);
		GetPrivateProfileString(strKey.c_str(), "AGING_TEST", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
		m_stSystem.bAging = (BOOL)atoi(str);

		// Unit is in seconds, default is one minute
		ZeroMemory(str);
		GetPrivateProfileString(strKey.c_str(), "AGING_DURATION", "60", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
		m_stSystem.bAging = (BOOL)atoi(str);


		GetPrivateProfileString(strKey.c_str(), "NAME", "ROBOT", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
		if (strlen(str) > 0 && strlen(str) < __MAX_PATH__)
			m_stSystem.strName = TSTRING(str);
		else
			m_stSystem.strName = "EtherCAT Slave";

		ZeroMemory(str);
		GetPrivateProfileString(strKey.c_str(), "EXTERNAL_INTERFACE_ENABLED", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
		m_stSystem.stExternalIface.bEnabled = (BOOL)atoi(str);

		ZeroMemory(str);
		GetPrivateProfileString(strKey.c_str(), "EXTERNAL_INTERFACE_PORT", "7420", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
		m_stSystem.stExternalIface.nPort = atoi(str);

		ZeroMemory(str);
		GetPrivateProfileString(strKey.c_str(), "TELEOPERATION_MODE", "0", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
		m_stSystem.nTeleMode = atoi(str);

		ZeroMemory(str);
		GetPrivateProfileString(strKey.c_str(), "TELEOPERATION_CONSTANT_SCALING", "0.3", str, (size_t)__MAX_PATH__, m_strConfigPath.c_str());
		m_stSystem.dTeleScalingRatio = atof(str);

	}
	catch (...)
	{
		DBG_LOG_ERROR("(%s) ReadSystemConfig - Exception Occurs!", "CConfigRobot");
		return FALSE;
	}

	return TRUE;
}

void
CConfigRobot::WriteLastPosition(INT32 anAxis, INT32 anRawPos)
{
	try 
	{
		TSTRING strKey = "AXIS" + TOSTRING(anAxis);
		TSTRING strValue = TOSTRING(anRawPos);
		WritePrivateProfileString(strKey.c_str(), "POS_BEFORE_EXIT", strValue.c_str(), m_strConfigPath.c_str());
	}
	catch (...)
	{
		DBG_LOG_ERROR("(%s) ReadSystemConfig - Exception Occurs!", "CConfigRobot");
		return;
	}
	return;
}

void
CConfigRobot::LoadDefaultConfig()
{
	return;
}