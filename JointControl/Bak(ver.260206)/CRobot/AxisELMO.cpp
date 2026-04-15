/*****************************************************************************
*	Name: AxisEPOS4.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implementation of the CAxisEPOS4 class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include "AxisELMO.h"
#include <algorithm>
#include <functional>
#include "commons.h"
#include "posix_rt.h"

/* adEncRes should include the encoding type of the encoder (X1, X2, X4)
* For joint types other than revolute, the adOneTurnRef should be given explicitly
* Ex) Screw-type prismatic joint, one turn is equal to the lead of the screw, thus adOneTurnRef should be equal to the lead (pitch * number of starts) of the screw.
*/
CAxisELMO::CAxisELMO(eAxisType aeAxisType, double adEncRes, double adAbsEncRes, double adGearRatio, double adTransRatio, BOOL abAbsoluteEncoder, BOOL abCCW, BOOL abEnabled, BOOL abConnected)
	: CAxis(eAxisEtherCAT, aeAxisType, abAbsoluteEncoder, abCCW, abEnabled)
{
	m_bConnected = abConnected;
	if (FALSE == m_bConnected) SetEnabled(FALSE);

	SetResolution(adEncRes, adAbsEncRes, adGearRatio, adTransRatio);

	if (eAxisRevolute != aeAxisType)
	{
		DBG_LOG_WARN("Axis type is not revolute, remember to set appropriate OneTurnRef");
	}

	// Initialize for single mode
	m_bIsQuartet = FALSE;
	m_btAxisIndex = 0;
	m_pQuartetSlave = nullptr;

	m_bFirstHome = TRUE;
	m_pEcMaster = NULL;
	m_nPosBeforeDisconnect = 0;
	m_bHomeSet = TRUE;
}

// Constructor for quartet axis - user only specifies axis index
CAxisELMO::CAxisELMO(eAxisType aeAxisType, double adEncRes, double adAbsEncRes, double adGearRatio, double adTransRatio, 
                     UINT8 abtAxisIndex, BOOL abAbsoluteEncoder, BOOL abCCW, BOOL abEnabled, BOOL abConnected)
    : CAxis(eAxisEtherCAT, aeAxisType, abAbsoluteEncoder, abCCW, abEnabled)
{
    if (abtAxisIndex > 3) {
        DBG_LOG_ERROR("(%s) Invalid quartet axis index %d - must be 0,1,2,3", "CAxisELMO", abtAxisIndex);
        abtAxisIndex = 0;
    }
    
    m_bConnected = abConnected;
    if (FALSE == m_bConnected) SetEnabled(FALSE);
    SetResolution(adEncRes, adAbsEncRes, adGearRatio, adTransRatio);

    if (eAxisRevolute != aeAxisType)
    {
        DBG_LOG_WARN("Axis type is not revolute, remember to set appropriate OneTurnRef");
    }

    // Initialize for quartet mode
    m_bIsQuartet = TRUE;
    m_btAxisIndex = abtAxisIndex;  // 0, 1, 2, or 3
    m_pQuartetSlave = nullptr;     // Will be set later when quartet slave is discovered

    m_bFirstHome = TRUE;
    m_pEcMaster = NULL;
    m_nPosBeforeDisconnect = 0;
    m_bHomeSet = TRUE;
}

CAxisELMO::~CAxisELMO()
{
	if (NULL != m_pEcMaster)
	{
		m_pEcMaster = NULL;
	}
}

/* Registers the EtherCAT master and initializes the slave. 
*  The axis will then attempt to change the CIA402 status to OPERATIONAL (Servo ON) and set the drive mode according to abtDrivemode.
*/
// ensure that the EtherCAT master instance is not changed
BOOL 
CAxisELMO::Init(CEcatMaster& apEcmaster, INT8 abtDriveMode)
{
	/* Pre-conditions 
	*  Each condition is separated to provide exact error type.
	*  todo: make this fucntion return int and provide error code?
	*/
	/* return TRUE when the axis is already initialized */
	if (eAxisInit <= GetState())	return TRUE;

	/* check if the axis limits are configured before initializing */
	if (FALSE == IsLimitConfigured())
	{
		DBG_LOG_ERROR("(%s) Axis Limits are not configured!", "CAxisEPOS4");
		DeInit();
		return FALSE;
	}

	eHomingMethod ehoming = GetHomingMethod();

	
	switch (ehoming)
	{
	case eAxisHomeStartPos:
		break;
	case eAxisHomeSearch:
		if (FALSE == IsHomeReferenceSet())
		{
			DBG_LOG_ERROR("(%s) HomeSearch Method is Selected, but Reference is not configured!", "CAxisEPOS4");
			DeInit();
			return FALSE;
		}
		break;
	case eAxisHomeManual:
		if (FALSE == IsHomeSet())
		{
			DBG_LOG_ERROR("(%s) HomeManual Method is Selected, but Home Position is not configured!", "CAxisELMO");
			DeInit();
			return FALSE;
		}
	case eAxisHomeBuiltin: // todo: consider this later
	case eAxisHomingNotSet:
	default:
		DBG_LOG_ERROR("(%s) Invalid Homing Method!", "CAxisELMO");
		DeInit();
		return FALSE;
	}

	if (TRUE == IsConnected())
	{
		m_pEcMaster = &apEcmaster;
		
		if (!m_bIsQuartet)
		{
			// Single axis - add slave to master normally
		m_pEcMaster->AddSlave(&m_cEcSlave);
		UINT16 usAlias = m_cEcSlave.GetAlias();
		UINT16 usPosition = m_cEcSlave.GetPosition();
		SetAliasPos(usAlias, usPosition);
		}
		else
		{
			// Quartet axis - slave should already be added to master by the quartet
			if (!m_pQuartetSlave)
			{
				DBG_LOG_ERROR("(%s) Quartet axis %d has no associated quartet slave!", "CAxisELMO", m_btAxisIndex);
				DeInit();
				return FALSE;
			}
			// Get alias/position from quartet slave
			UINT16 usAlias = m_pQuartetSlave->GetAlias();
			UINT16 usPosition = m_pQuartetSlave->GetPosition();
			SetAliasPos(usAlias, usPosition);
		}

		if (TRUE == IsEnabled())
		{
			if (!m_bIsQuartet)
			{
				// Single axis - register callbacks directly with the CIA402 slave
				m_cEcSlave.RegisterCallbackStateChange(std::bind(&CAxisELMO::OnSlaveStatusChanged, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
				m_cEcSlave.RegisterCallbackParamUpdate(std::bind(&CAxisELMO::OnSlaveUpdateRawParam, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
			}
			else if (m_pQuartetSlave)
			{
				// Quartet axis - register per-axis callbacks with the quartet slave
				m_pQuartetSlave->RegisterCallbackStateChange(m_btAxisIndex, 
					std::bind(&CAxisELMO::OnSlaveStatusChanged, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
				m_pQuartetSlave->RegisterCallbackParamUpdate(m_btAxisIndex,
					std::bind(&CAxisELMO::OnSlaveUpdateRawParam, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
				DBG_LOG_INFO("(%s) Quartet axis %d callbacks registered", "CAxisELMO", m_btAxisIndex);
			}

			BOOL bRet = ServoOn(abtDriveMode);
			if (TRUE == bRet)
			{
				TSTRING szDriveMode = GetCIA402DriveMode((int)abtDriveMode);
				TSTRING szAxisType = m_bIsQuartet ? "Quartet" : "Single";
				DBG_LOG_INFO("(%s) %s Axis[%d:%d] Initialized! DriveMode: %s", "CAxisELMO", szAxisType.c_str(), m_usAxisAlias, m_usAxisPos, szDriveMode.c_str());
				return CAxis::Init(); // change the state to eAxisInit
			}
			else
			{
				DBG_LOG_WARN("(%s) Axis[%d:%d] Initialized, but ServoOn Failed!", "CAxisELMO", m_usAxisAlias, m_usAxisPos);
				DeInit();
				return FALSE; 
			}
		}
		DBG_LOG_WARN("(%s) Axis[%d:%d] Initialized, but Axis is Disabled!", "CAxisELMO", m_usAxisAlias, m_usAxisPos);
		return CAxis::Init(); // change the state to eAxisInit
	}
	DBG_LOG_WARN("(%s) Axis is not connected!", "CAxisELMO");
	return DeInit(); // return TRUE but change the state to eAxisDeInit
}

BOOL 
CAxisELMO::DeInit()
{
	return CAxis::DeInit();
}

BOOL
CAxisELMO::DoHoming()
{
	return FALSE;
}

/* radians */
BOOL
CAxisELMO::MoveAxis(double adTarget, BOOL abForce, BOOL abAbs)
{
	/* only allow this when in any of the position-based operation mode 
	*/
	if ((CIA402_CYCLIC_POSITION != RouteGetActualDriveMode()) && (CIA402_PROFILE_POSITION != RouteGetActualDriveMode()))
	{
		DBG_LOG_WARN("(%s) Axis[%d:%d] Move Axis - Not Position Mode!", "CAxisELMO", m_usAxisAlias, m_usAxisPos);
		return FALSE;
	}
	
	/* check if limits are configured and if the status is IDLE 
	*  bypass this condition when forced.
	*/
	if ((FALSE == abForce) && (FALSE == IsMovable()))
	{
		DBG_LOG_WARN("(%s) Axis[%d:%d] Move Axis - Not Moveable!", "CAxisELMO", m_usAxisAlias, m_usAxisPos);
		return FALSE;
	}
		
	/*
	* Absolute motion is only allowable when home position is configured
	*/
	INT32 nReferencePos = 0;
	if (TRUE == abAbs && TRUE == IsHomeSet())
	{
		
		nReferencePos = GetHomePosition();
		if (FALSE == IsAllowablePosition(adTarget)) 
		{
			DBG_LOG_WARN("(%s) Axis[%d:%d] Move Axis - Exceeded Limit! Target: %lf, Cur Pos: %lf, LL: %lf, UL: %lf", "CAxisELMO", m_usAxisAlias, m_usAxisPos, adTarget, GetCurrentPos(), GetAxisLimits().stPos.dLower, GetAxisLimits().stPos.dUpper);
			return FALSE;
		}
	}
	else
	{
		
		nReferencePos = GetCurrentRawPos();
		if (TRUE == IsHomeSet())
		{
			if (FALSE == IsAllowablePosition(GetCurrentPos() + adTarget))
			{
				DBG_LOG_WARN("(%s) Master[%d], Axis[%d:%d] Move Axis - Exceeded Limit!", "CAxisELMO",m_pEcMaster->GetMasterID(), m_usAxisAlias, m_usAxisPos);
				return FALSE;
			}
		}
	}
	
	m_dTargetPos = adTarget;
	

	
	DBG_LOG_WARN("(%s) Axis[%d:%d] Move Axis - TargetPos! %d %d", "CAxisELMO", m_usAxisAlias, m_usAxisPos, nReferencePos, nReferencePos + ConvertRadMM2Res(adTarget));
	return RouteMovePosition(adTarget, TRUE);
	
}

BOOL
CAxisELMO::MoveHome(BOOL abForce)
{
	/*todo: shall we add force here? */
	if ((FALSE == IsMovable())) return FALSE;

	/* only allow this when in any of the position-based operation mode */
	if ((CIA402_CYCLIC_POSITION != RouteGetActualDriveMode()) && (CIA402_PROFILE_POSITION != RouteGetActualDriveMode()))
		return FALSE;

	return RouteMovePosition(GetHomePosition(), abForce);
	// return TRUE;
}

BOOL
CAxisELMO::StopAxis()
{
	if (FALSE == IsMovable()) return FALSE;

	return RouteStopAxis();
}

BOOL
CAxisELMO::EmgStopAxis()
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		// Note: CElmoQuartet doesn't have SetEmgStop method yet
		DBG_LOG_WARN("(%s) SetEmgStop not yet implemented for quartet axis %d", "CAxisELMO", m_btAxisIndex);
		return FALSE;
	}
	else
	{
		return m_cEcSlave.SetEmgStop();
	}
}

/* rad/s */
BOOL
CAxisELMO::MoveVelocity(double adVel)
{
	if ((FALSE == IsMovable()) || (FALSE == IsAllowableVelocity(adVel))) return FALSE;

	/* only allow this when in any of the velocity-based operation mode */
	if ((CIA402_CYCLIC_VELOCITY != RouteGetActualDriveMode()) && (CIA402_PROFILE_VELOCITY != RouteGetActualDriveMode()))
		return FALSE;

	return RouteMoveVelocity(adVel);
}

BOOL
CAxisELMO::MoveTorque(double adTor) // adTor::출력단[Nm]
{

	
	if (FALSE == IsMovable()) {
		DBG_LOG_WARN("(%s) Master[%d], Axis[%d:%d] NOT is Movablel", "CAxisELMO", m_pEcMaster->GetMasterID(), m_usAxisAlias, m_usAxisPos);
		return FALSE;

	}
	

	// printf("axis:%d, target torq: %f\n", m_usAxisPos, adTor);
	/* only allow this when in any of the torque-based operation mode */
	if ((CIA402_CYCLIC_TORQUE != RouteGetActualDriveMode()) && (CIA402_PROFILE_TORQUE != RouteGetActualDriveMode()))
		return FALSE;
	
	m_dTargetTorq = adTor;

	// 모터 입력단 토크로 변환
	double dTor = adTor/GetGearRatio();	

	return RouteMoveTorque(dTor);

	// if (!IsAllowableTorque(dTor)) {	
	// 	RouteMoveTorque(0.0);
		
	// 	DBG_LOG_WARN("(%s) Master[%d], Axis[%d:%d] Torque Limited", "CAxisELMO", m_pEcMaster->GetMasterID(), m_usAxisAlias, m_usAxisPos);
	// 	return FALSE;
	// }

	
	// if (IsAllowablePosition(GetCurrentPos())) {
		
	// 	return RouteMoveTorque(dTor);
		
	// }
	// else {		
		
	// 	DBG_LOG_WARN("(%s) Master[%d], Axis[%d:%d] Move Torque - Exceeded Position Limit!", "CAxisELMO", m_pEcMaster->GetMasterID(),m_usAxisAlias, m_usAxisPos);		
	// 	return FALSE;
	// }
	

	return TRUE;
}

BOOL
CAxisELMO::ServoOn(INT8 abtDriveMode)
{
	if ((FALSE == IsConnected()) && (FALSE == IsEnabled())) return FALSE;

	return RouteServoOn(abtDriveMode);
}

BOOL
CAxisELMO::ChangeDriveMode(INT8 abtDriveMode)
{
	// if (FALSE == IsServoOn()) return FALSE;

	if (m_bIsQuartet && m_pQuartetSlave)
	{
		m_pQuartetSlave->SetDriveMode(m_btAxisIndex, abtDriveMode);
	}
	else
	{
	m_cEcSlave.SetDriveMode(abtDriveMode);
	}

	return TRUE;
}

BOOL
CAxisELMO::ServoOff(	)
{
	return RouteServoOff();
}

BOOL
CAxisELMO::IsServoOn()
{
	if (m_bIsQuartet && m_pQuartetSlave) {
		return m_pQuartetSlave->IsServoOn(m_btAxisIndex);
	}
	return m_cEcSlave.IsServoOn();
}

BOOL	
CAxisELMO::SetVelocity(double adRPM)
{
	if (FALSE == IsLimitConfigured() || FALSE == IsAllowableVelocity(adRPM))
		return FALSE;
	
	//check unit used in EPOS4
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		
		// m_pQuartetSlave->SetProfileVel(m_btAxisIndex, (INT32)adRPM);
		// m_pQuartetSlave->SetMaxProfileVel(m_btAxisIndex, (INT32)adRPM);
	}
	else
	{
	m_cEcSlave.SetProfileVel((INT32)adRPM);
	m_cEcSlave.SetMaxProfileVel((INT32)adRPM);
	}

	return TRUE;
}

BOOL	
CAxisELMO::SetAcceleration(double adRPMS)
{
	if (FALSE == IsLimitConfigured() || FALSE == IsAllowableAcceleration(adRPMS))
		return FALSE;

	if (m_bIsQuartet && m_pQuartetSlave)
	{
		// m_pQuartetSlave->SetProfileAcc(m_btAxisIndex, (INT32)adRPMS);
	}
	else
	{
	m_cEcSlave.SetProfileAcc((INT32)adRPMS);
	}
	return TRUE;
}

BOOL	
CAxisELMO::SetDeceleration(double adRPMS)
{
	if (FALSE == IsLimitConfigured() || FALSE == IsAllowableDeceleration(adRPMS))
		return FALSE;

	if (m_bIsQuartet && m_pQuartetSlave)
	{
		// m_pQuartetSlave->SetProfileDec(m_btAxisIndex, (INT32)adRPMS);
	}
	else
	{
	m_cEcSlave.SetProfileDec((INT32)adRPMS);
	}
	return TRUE;
}

INT32
CAxisELMO::GetAcceleration()
{
	return RouteGetActualProfileAcc();
}

INT32
CAxisELMO::GetDeceleration()
{
	return RouteGetActualProfileDec();
}

INT32
CAxisELMO::GetTargetRawPos()
{
	return 0;
	// return RouteGetTargetPos();
}

INT32
CAxisELMO::GetTargetRawVel()
{
	return RouteGetTargetVel();
}

INT32
CAxisELMO::GetTargetRawTor()
{
	
	return (INT32)RouteGetTargetTor();
}

INT32
CAxisELMO::GetVelocity()
{
	return RouteGetActualProfileVel();
}

INT32
CAxisELMO::GetMaxVel()
{
	return RouteGetActualMaxProfileVel();
}

INT32
CAxisELMO::GetMaxAcc()
{
	return RouteGetActualMaxProfileAcc();
}

double CAxisELMO::GetAdditionalPos()
{
	int nPos = NormalizeAbsPosition((int)RouteGetAdditionalPos(), m_dAbsResolution);
	
	// double dAdditionalPos = nPos * m_dOneTurnRef / m_dAbsResolution * m_nDirection;
	double dAdditionalPos = ConvertRes2RadMMAbs(nPos);
	return dAdditionalPos;
}

INT32
CAxisELMO::GetQuickStopDec()
{
	return RouteGetActualQuickStopDec();
}

UINT8
CAxisELMO::GetDriveMode()
{
	return RouteGetActualDriveMode();
}

UINT16
CAxisELMO::GetStatusWord()
{
	return RouteGetStatusWord();
}

void	
CAxisELMO::SetVendorInfo(UINT32 auVendorID, UINT32 auProductCode)
{
	if (!m_bIsQuartet)
{
	m_cEcSlave.SetVendorInfo(auVendorID, auProductCode);
	}
	// For quartet axes, vendor info is set on the shared quartet slave, not here
}
void	
CAxisELMO::SetDCInfo(BOOL abDCSupported, UINT16 ausActivateWord, INT32 anShiftTime)
{
	if (!m_bIsQuartet)
{
	m_cEcSlave.SetDCInfo(abDCSupported, ausActivateWord, anShiftTime);
	}
	// For quartet axes, DC info is set on the shared quartet slave, not here
}

UINT16
CAxisELMO::GetControlWord()
{
	return RouteGetControlWord();
}

void 
CAxisELMO::SetState(eAxisState aeState)
{
	TSTRING			szState = "";
	BOOL			bPrintState = FALSE;
	eAxisState ePrevState = GetState();
	eAxisState eCurrState = aeState;
	m_pstAxisInfo->eState = eCurrState;

	if (ePrevState != eCurrState)
	{
		/* in case disconnected due to power loss
		*/
		if (eAxisDisconnected == ePrevState)
		{
			/* todo: in-position error should be considered */
			if (GetCurrentRawPos() != m_nPosBeforeDisconnect)
			{
				// ClearHomeSet();
				return;
			}
		}

		// if (eAxisError == ePrevState)
		// {
		// 	DBG_LOG_TRACE("(%s) Axis[%d:%d] Fault Reset: TTTTargetPos: %d, RRRRawPos:%d", "CAxisEPOS4", m_usAxisAlias, m_usAxisPos, GetTargetRawPos(), GetCurrentRawPos());
		// }
		
		switch (eCurrState)
		{
		case eAxisEmergency:
			szState = "EMERGENCY";
			break;
		case eAxisError:
			// DBG_LOG_TRACE("(%s) Axis[%d:%d] Fault : TTTTargetPos: %d, RRRRawPos:%d", "CAxisEPOS4", m_usAxisAlias, m_usAxisPos, GetTargetRawPos(), GetCurrentRawPos());
			szState = "ERROR";
			break;
		case eAxisDisconnected:
			szState = "DISCONNECTED";
			m_nPosBeforeDisconnect = GetCurrentRawPos(); 
			break;
		case eAxisDeinit:
			szState = "DEINIT";
			break;
		case eAxisInit:
			szState = "INIT";
			break;
		case eAxisIdle:
			szState = "IDLE";
			break;
		case eAxisHoming:
			szState = "HOMING";
			break;
		case eAxisRunning:
			szState = "RUNNING";
			break;
		case eAxisStopped:
			szState = "STOPPED";
			break;
		case eAxisUnknownState:
		default:
			szState = "UNKNOWN";
			break;
		}
		/* do not print state change IDLE<->RUNNING because it may flood STDOUT */
		if (bPrintState)
		{
			DBG_LOG_INFO("(%s) Axis[%d:%d] Current State is : %s", "CAxisEPOS4", m_usAxisAlias, m_usAxisPos, szState.c_str());

			if (eAxisDisconnected == ePrevState)
			{
				DBG_LOG_INFO("(%s) Axis[%d:%d] Connection recovered!", "CAxisEPOS4", m_usAxisAlias, m_usAxisPos);
			}
		}
	}
}

void 
CAxisELMO::SetHomingFlag(BOOL abFlag)
{
	m_bIsHoming = abFlag;
	if (TRUE == abFlag)
		SetState(eAxisHoming);
}


/* Callback function for servo status change */
void
CAxisELMO::OnSlaveStatusChanged(PVOID apSlave, PVOID apStatus, PVOID apReserved1, PVOID apReserved2)
{
	eSERVO_STATUS	eServoStatus = *(eSERVO_STATUS*)apStatus;

	// For quartet axes, apReserved1 contains the axis index
	UINT8 axisIndex = 0;
	if (m_bIsQuartet && apReserved1) {
		axisIndex = *(UINT8*)apReserved1;
		if (axisIndex != m_btAxisIndex) {
			// This callback is not for this axis
			return;
		}
	}
	
	switch (eServoStatus)
	{
	case eServoOff:
		//SetState(eAxisInit);
		// 
		break;
	case eServoIdle:
		
        // m_bFirstHome = FALSE; 
		if (TRUE == m_bFirstHome)
		{
			INT32 nStartPos, nStartVel, nStartTor, nAbsPos;
			
			// Get start position from appropriate slave type
			if (m_bIsQuartet && m_pQuartetSlave) {
				nStartPos = m_pQuartetSlave->GetActualPos(m_btAxisIndex);
				nStartVel = m_pQuartetSlave->GetActualVel(m_btAxisIndex);
				nStartTor = m_pQuartetSlave->GetActualTor(m_btAxisIndex);
				nAbsPos = (INT32)m_pQuartetSlave->GetAdditionalPos(m_btAxisIndex);
				
			} else {
				nStartPos = m_cEcSlave.GetActualPos();
				nStartVel = m_cEcSlave.GetStartVel();
				nStartTor = m_cEcSlave.GetStartTor();
				nAbsPos = (INT32)m_cEcSlave.GetAdditionalPos();
				
			}
			
            DBG_LOG_INFO("nStartPos:%d (axis %d)\n", nStartPos, m_bIsQuartet ? m_btAxisIndex : 0);
			if (eAxisHomeStartPos == GetHomingMethod())
			{				
				
				// SetHomePosition(nStartPos - GetPositionBeforeExit());
			}
			UpdateCurrentParams(nStartPos, nStartVel, nStartTor, nAbsPos);
			/* todo: do we need this? */
			UpdateStartRawParams(m_bIsQuartet, m_btAxisIndex, nStartPos, nStartVel, nStartTor, nAbsPos);
			
			m_bFirstHome = FALSE;
		}
		if (eAxisHoming == GetState())
			m_bIsHoming = FALSE;

		if (TRUE == IsHomeSet())
			SetState(eAxisIdle);
		else
			SetState(eAxisInit);
		break;
	case eServoHoming:
		SetState(eAxisHoming);
		break;
	case eServoRunning:
		SetState(eAxisRunning);
		break;
	case eServoStopped:
		SetState(eAxisStopped);
		break;
	case eServoFault:
		SetState(eAxisError);
		break;
	case eServoDisconnected:
		SetState(eAxisDisconnected);
		break;
	default:
		break;
	}
}

/* Callback function for updating raw data from the servo driver.
*  This will be called everytime a new EtherCAT packet has been parsed.
*/
void
CAxisELMO::OnSlaveUpdateRawParam(PVOID apSlave, PVOID apParams, PVOID apStatus, PVOID apReserved1)
{
	INT32 nActualPos, nActualVel, nActualTor, nActualAbsPos;
	
	if (m_bIsQuartet && m_pQuartetSlave) {
		// For quartet axes, apReserved1 contains the axis index
		UINT8 axisIndex = 0;
		if (apReserved1) {
			axisIndex = *(UINT8*)apReserved1;
			if (axisIndex != m_btAxisIndex) {
				// This callback is not for this axis
				return;
			}
		}
		
		// Get data directly from quartet slave for this axis
		nActualPos = m_pQuartetSlave->GetActualPos(m_btAxisIndex);
		nActualVel = m_pQuartetSlave->GetActualVel(m_btAxisIndex);
		nActualTor = m_pQuartetSlave->GetActualTor(m_btAxisIndex);
		nActualAbsPos = m_pQuartetSlave->GetAdditionalPos(m_btAxisIndex);
	} else {
		// Single axis - get data from raw data structure
		ST_RAW_DATA stRawParams = *(ST_RAW_DATA*)apParams;
		nActualPos = stRawParams.nActualPos;
		nActualVel = stRawParams.nActualVel;
		nActualTor = stRawParams.nActualTor;
		nActualAbsPos = (int)stRawParams.dAdditionalPos;
	}

	UpdateCurrentParams(nActualPos, nActualVel, nActualTor, nActualAbsPos);
}

BOOL CAxisELMO::CheckAxisLimits()
{
	ST_AXIS_LIMITS stLimits = GetAxisLimits();

	BOOL bLimited = FALSE;
	// bLimited = IsAllowablePosition(GetCurrentPos());
	// if (bLimited == FALSE)
	// {
		
		
	// 	DBG_LOG_WARN("(Axis[%d:%d] Exceeded Position Limit! Pos: %lf, LL: %lf, UL: %lf", 
	// 		m_usAxisAlias, m_usAxisPos, m_pstAxisParams->dPos, stLimits.stPos.dLower, stLimits.stPos.dUpper);

	// 	return FALSE;
		
	// 	// if (IsMoving() && !IsAllowablePosition(GetCurrentPos())) {
	// 	// 	if (IsAllowablePosition(GetTargetPos()) && IsAllowableTorque(GetTargetTorq()/GetGearRatio())) {
	// 	// 		return TRUE;
	// 	// 	}
	// 	// 	return FALSE;
	// 	// }

	// }


	// bLimited = CheckLimits(stLimits.stVel, m_pstAxisParams->dVel);
	// if (bLimited == FALSE)
	// {
	// 	// if (IsMoving())
	// 	// 	StopAxis();
			
	// 	DBG_LOG_WARN("(Axis[%d:%d] Exceeded Velocity Limit! Vel: %lf, LL: %lf, UL: %lf", 
	// 		m_usAxisAlias, m_usAxisPos, m_pstAxisParams->dVel, stLimits.stVel.dLower, stLimits.stVel.dUpper);

	// 	return bLimited;
	// }

	if (m_usAxisPos == 5 || m_usAxisPos == 6 || m_usAxisPos == 7 
		|| m_usAxisPos == 12 || m_usAxisPos == 13 || m_usAxisPos == 14) {
		return TRUE;
	}
	else {
		bLimited = IsAllowableTorque(GetCurrentTor()/GetGearRatio());
		if (bLimited == FALSE)
		{
			// if (IsMoving()) // && !IsAllowableTorque(GetTargetTorq()))
			// 	StopAxis();
			
			DBG_LOG_WARN("(Axis[%d:%d] Exceeded Torque Limit! Torq: %lf, LL: %lf, UL: %lf", 
				m_usAxisAlias, m_usAxisPos, m_pstAxisParams->dTor, stLimits.stTorque.dLower, stLimits.stTorque.dUpper);

			return FALSE;
			
			// if (IsMoving() && (!IsAllowableTorque(GetCurrentTor()))) {
			// 	if (IsAllowablePosition(GetTargetPos()) && IsAllowableTorque(GetTargetTorq()/GetGearRatio())) {				
			// 		return TRUE;
			// 	}
				
			// 	return FALSE;;
			// }
		}	

		return TRUE;
	}
	
}

INT16
CAxisELMO::ConvertTor2Res(double adTor)
{
	INT16 res = (INT16)(ConvertTor2Cur(adTor) * 1000.0 / GetRatedCurrent() * m_nDirection);
	
	return  (INT16)res;
}

double
CAxisELMO::ConvertRes2Tor(INT32 adRes)
{
	// DBG_LOG_INFO("CAxisELMO::ConvertRes2Tor");
	double torque = adRes * GetRatedTorque() / 1000.0 * m_nDirection * GetGearRatio();
	
	return torque;
}

// Factory method - creates all 4 axes for a quartet
std::vector<std::unique_ptr<CAxisELMO>> CAxisELMO::CreateQuartetAxes(
    eAxisType aeAxisType, double adEncRes, double adGearRatio, double adTransRatio,
    BOOL abAbsoluteEncoder, BOOL abCCW, BOOL abEnabled, BOOL abConnected)
{
    std::vector<std::unique_ptr<CAxisELMO>> axes;
    
    // Create 4 axes - user doesn't specify EtherCAT positions
    for (UINT8 axisIndex = 0; axisIndex < 4; axisIndex++)
    {
        auto axis = std::make_unique<CAxisELMO>(aeAxisType, adEncRes, adGearRatio, adTransRatio, 
                                               axisIndex, abAbsoluteEncoder, abCCW, abEnabled, abConnected);
        axes.push_back(std::move(axis));
    }
    
    return axes;
}

// Method to associate axis with quartet slave (called internally by EtherCAT framework)
void CAxisELMO::SetQuartetSlave(std::shared_ptr<CElmoQuartet> apQuartetSlave)
{
	if (!m_bIsQuartet) {
		DBG_LOG_ERROR("(%s) Cannot set quartet slave on single axis!", "CAxisELMO");
		return;
	}
	
	m_pQuartetSlave = apQuartetSlave;
	DBG_LOG_INFO("(%s) Quartet axis %d associated with slave", "CAxisELMO", m_btAxisIndex);
}

// Override parameter update to handle both single and quartet axes
void CAxisELMO::UpdateAxisParameters()
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		// For quartet axes, get values directly from quartet slave
		INT32 nCurrentPos = RouteGetCurrentPos();
		INT32 nCurrentVel = RouteGetCurrentVel();
		INT32 nCurrentTor = RouteGetCurrentTor();
		INT32 nCurrentAbsPos = (INT32)RouteGetAdditionalPos();
		
		UpdateCurrentParams(nCurrentPos, nCurrentVel, nCurrentTor, nCurrentAbsPos);
	}
	else
	{
		// For single axes, the callback mechanism handles updates automatically
		// This method is mainly for manual updates when needed
		INT32 nCurrentPos = m_cEcSlave.GetActualPos();
		INT32 nCurrentVel = m_cEcSlave.GetActualVel();
		INT32 nCurrentTor = m_cEcSlave.GetActualTor();
		INT32 nCurrentAbsPos = (INT32)m_cEcSlave.GetAdditionalPos();
		
		UpdateCurrentParams(nCurrentPos, nCurrentVel, nCurrentTor, nCurrentAbsPos);
	}
}

// Helper routing methods for device type abstraction
BOOL CAxisELMO::RouteServoOn(INT8 abtDriveMode)
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		return m_pQuartetSlave->SetServoOn(m_btAxisIndex, abtDriveMode);
	}
	else
	{
		return m_cEcSlave.SetServoOn(abtDriveMode);
	}
}

BOOL CAxisELMO::RouteServoOff()
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		return m_pQuartetSlave->SetServoOff(m_btAxisIndex);
	}
	else
	{
		return m_cEcSlave.SetServoOff();
	}
}

BOOL CAxisELMO::RouteMoveTorque(double adTorque)
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		m_pQuartetSlave->SetTargetTor(m_btAxisIndex, (INT16)ConvertTor2Res(adTorque));
		return TRUE;
	}
	else
	{
		m_cEcSlave.SetTargetTor((INT16)ConvertTor2Res(adTorque));
		return TRUE;
	}
}

BOOL CAxisELMO::RouteMovePosition(double adPosition, BOOL abAbs)
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		// Note: CElmoQuartet doesn't have SetTargetPos yet, but the pattern is ready
		// m_pQuartetSlave->SetTargetPos(m_btAxisIndex, (INT32)ConvertPos2Res(adPosition, abAbs));
		DBG_LOG_WARN("(%s) Position mode not yet implemented for quartet axis %d", "CAxisELMO", m_btAxisIndex);
		return FALSE;
	}
	else
	{
		printf("Axis: %d, nOffset: %d\n", m_usAxisPos, m_nOffsetPos);
		return m_cEcSlave.SetTargetPos((INT32)ConvertRadMM2Res(adPosition)+m_nOffsetPos);
	}
}

BOOL CAxisELMO::RouteMoveVelocity(double adVelocity)
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		// Note: CElmoQuartet doesn't have SetTargetVel yet, but the pattern is ready
		// m_pQuartetSlave->SetTargetVel(m_btAxisIndex, (INT32)ConvertVel2Res(adVelocity));
		DBG_LOG_WARN("(%s) Velocity mode not yet implemented for quartet axis %d", "CAxisELMO", m_btAxisIndex);
		return FALSE;
	}
	else
	{
		m_cEcSlave.SetTargetVel((INT32)ConvertRadMM2Res(adVelocity));
		return TRUE;
	}
}

BOOL CAxisELMO::RouteStopAxis()
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		return m_pQuartetSlave->SetEmgStop(m_btAxisIndex);
	}
	else
	{
		return m_cEcSlave.SetHalt(TRUE);
	}
}

UINT16 CAxisELMO::RouteGetStatusWord()
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		return m_pQuartetSlave->GetRawStatusWord(m_btAxisIndex);
	}
	else
	{
		return m_cEcSlave.GetRawStatusWord();
	}
}

INT32 CAxisELMO::RouteGetCurrentPos()
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		return m_pQuartetSlave->GetActualPos(m_btAxisIndex);
	}
	else
	{
		return m_cEcSlave.GetActualPos();
	}
}

INT32 CAxisELMO::RouteGetCurrentVel()
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		return m_pQuartetSlave->GetActualVel(m_btAxisIndex);
	}
	else
	{
		return m_cEcSlave.GetActualVel();
	}
}

INT32 CAxisELMO::RouteGetCurrentTor()
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		return m_pQuartetSlave->GetActualTor(m_btAxisIndex);
	}
	else
	{
		return m_cEcSlave.GetActualTor();
	}
}

UINT8 CAxisELMO::RouteGetActualDriveMode()
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		return m_pQuartetSlave->GetActualDriveMode(m_btAxisIndex);
	}
	else
	{
		return m_cEcSlave.GetActualDriveMode();
	}
}

INT32 CAxisELMO::RouteGetActualProfileAcc()
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		return 0;
		// return m_pQuartetSlave->GetActualProfileAcc(m_btAxisIndex);
	}
	else
	{
		return m_cEcSlave.GetActualProfileAcc();
	}
}

INT32 CAxisELMO::RouteGetActualProfileDec()
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		return 0;
		// return m_pQuartetSlave->GetActualProfileDec(m_btAxisIndex);
	}
	else
	{
		return m_cEcSlave.GetActualProfileDec();
	}
}

INT32 CAxisELMO::RouteGetTargetVel()
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		return 0;
		// return m_pQuartetSlave->GetTargetVel(m_btAxisIndex);
	}
	else
	{
		return m_cEcSlave.GetTargetVel();
	}
}

INT16 CAxisELMO::RouteGetTargetTor()
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		return m_pQuartetSlave->GetTargetTor(m_btAxisIndex);
	}
	else
	{
		return m_cEcSlave.GetTargetTor();
	}
}

UINT32 CAxisELMO::RouteGetActualProfileVel()
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		return 0;
		// return m_pQuartetSlave->GetActualProfileVel(m_btAxisIndex);
	}
	else
	{
		return m_cEcSlave.GetActualProfileVel();
	}
}

UINT32 CAxisELMO::RouteGetActualMaxProfileVel()
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		return 0;
		// return m_pQuartetSlave->GetActualMaxProfileVel(m_btAxisIndex);
	}
	else
	{
		return m_cEcSlave.GetActualMaxProfileVel();
	}
}

UINT32 CAxisELMO::RouteGetActualMaxProfileAcc()
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		// return m_pQuartetSlave->GetActualMaxProfileAcc(m_btAxisIndex);
		return 0; // TODO: Implement GetActualMaxProfileAcc in CElmoQuartet
	}
	else
	{
		return m_cEcSlave.GetActualMaxProfileAcc();
	}
}

UINT32 CAxisELMO::RouteGetActualQuickStopDec()
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		return 0;
		// return m_pQuartetSlave->GetActualQuickStopDec(m_btAxisIndex);
	}
	else
	{
		return m_cEcSlave.GetActualQuickStopDec();
	}
}

UINT16 CAxisELMO::RouteGetControlWord()
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		return m_pQuartetSlave->GetControlWord(m_btAxisIndex);
	}
	else
	{
		return m_cEcSlave.GetControlWord();
	}
}

double CAxisELMO::RouteGetAdditionalPos()
{
	if (m_bIsQuartet && m_pQuartetSlave)
	{
		return m_pQuartetSlave->GetAdditionalPos(m_btAxisIndex);
	}
	else
	{
		return m_cEcSlave.GetAdditionalPos();
	}
}

void CAxisELMO::UpdateHomeOffset()
{
			
	INT32 nPos, nAbsPos;
	
	// Get start position from appropriate slave type
	if (m_bIsQuartet && m_pQuartetSlave) {
		nPos = m_pQuartetSlave->GetActualPos(m_btAxisIndex);		
		nAbsPos = (INT32)m_pQuartetSlave->GetAdditionalPos(m_btAxisIndex);		
	} else {
		nPos = m_cEcSlave.GetActualPos();		
		nAbsPos = (INT32)m_cEcSlave.GetAdditionalPos();		
	}
	
	DBG_LOG_INFO("nStartPos:%d (axis %d)\n", nPos, m_bIsQuartet ? m_btAxisIndex : 0);
	

	nAbsPos = NormalizeAbsPosition(nAbsPos, m_dAbsResolution);	

	INT32 nIncPos = (INT32)(nAbsPos*m_dResolution/m_dAbsResolution*m_dGearRatio);	

	// if (m_strName == "L_HIP_PITCH" || m_strName == "R_HIP_PIsTCH") {
	// 	m_nOffsetPos = 0 - nPos;
	// }
	// else
	m_nOffsetPos = nIncPos - nPos;	
	

	if (m_strName == "L_ShoulderPitch"
		|| m_strName == "R_ShoulderPitch" 
		|| m_strName == "L_ShoulderRoll"
		|| m_strName == "R_ShoulderRoll"
		|| m_strName == "L_Wrist1" || m_strName == "L_Wrist2"		
		|| m_strName == "R_Wrist1" || m_strName == "R_Wrist2"
		|| m_strName == "L_ElbowPitch" || m_strName == "R_ElbowPitch") 
	{
		m_nOffsetPos = 0 - nPos;	
	}
	else {
		m_nOffsetPos = nIncPos - nPos;	
	}
			
		
		
}