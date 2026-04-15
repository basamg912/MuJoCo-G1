/*****************************************************************************
*	Name: Axis.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implementation of the CAxis class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include "Axis.h"


/////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
CAxis::CAxis(eCommType aeCommType, eAxisType aeAxisType, BOOL abAbsoluteEncoder, BOOL abCCW, BOOL abEnabled)
{
	m_bEnabled = abEnabled;
	m_pstAxisInfo = new ST_AXIS_INFO();
	m_pstAxisLimits = new ST_AXIS_LIMITS();
	m_pstAxisParams = new ST_AXIS_PARAMS();
	m_pstAxisRawParams = new ST_AXIS_RAW_PARAMS();
	m_pstAxisStartRawParams = new ST_AXIS_RAW_PARAMS();

	SetCommType(aeCommType);
	SetAxisType(aeAxisType);
	SetState(eAxisUnknownState);

	m_dResolution = 1.;
	m_dAbsResolution = 1.0;
	m_dGearRatio = 1.;
	m_dEncoderRatio = 1.;
	
	m_dTorqueConstant = 1.;

	m_nMasterID = 0;
	m_usAxisAlias = 0;
	m_usAxisPos = 0;
	m_usAxisID = 0;
	
	m_nHomePosition = 0;
	m_bHomeSet = FALSE;
	m_bHomeRefSet = FALSE;

	/* Call the method SetOneTurnRef if the axis is not revolute */
	m_dOneTurnRef = (eAxisRevolute == aeAxisType) ? TWO_M_PI : 1.;
	
	/* normal polarity = CCW, inverse polarity = CW */
	m_nDirection = 1;
	if (FALSE == abCCW) m_nDirection = -1;

	m_bAbsEncoder = abAbsoluteEncoder;
	m_dHomeReference = 0.;
	m_nPosBeforeExit = 0;
	m_bIsHoming = FALSE;

	m_dTargetTorq = 0.;
	m_dTargetPos = 0.;
	m_dTargetVel = 0.;
}

CAxis::~CAxis()
{
	if (m_pstAxisInfo)
	{
		delete m_pstAxisInfo;
		m_pstAxisInfo = NULL;
	}

	if (m_pstAxisLimits)
	{
		delete m_pstAxisLimits;
		m_pstAxisLimits = NULL;
	}

	if (m_pstAxisParams)
	{
		delete m_pstAxisParams;
		m_pstAxisParams = NULL;
	}

	if (m_pstAxisRawParams)
	{
		delete m_pstAxisRawParams;
		m_pstAxisRawParams = NULL;
	}

	if (m_pstAxisStartRawParams)
	{
		delete m_pstAxisStartRawParams;
		m_pstAxisStartRawParams = NULL;
	}
}
//////////////////////////////////////////////////////////////////////
BOOL
CAxis::Init()
{

	SetState(eAxisInit);
	return TRUE;
}

BOOL
CAxis::DeInit()
{
	SetState(eAxisDeinit);
	return TRUE;
}

BOOL 
CAxis::InitHW()
{

	return FALSE;
}

BOOL 
CAxis::InitSW()
{

	return FALSE;
}

BOOL 
CAxis::DeInitHW()
{

	return FALSE;
}

BOOL 
CAxis::DeinitSW()
{

	return FALSE;
}

BOOL
CAxis::ServoOn(	)
{
	return FALSE;
}

BOOL
CAxis::ServoOff()
{
	return FALSE;
}

BOOL
CAxis::DoHoming()
{
	return FALSE;
}

BOOL
CAxis::MoveHome(BOOL abForced)
{
	return MoveAxis(m_nHomePosition, abForced);
}

BOOL	
CAxis::MovePosLimit(BOOL abForced)
{
	return MoveAxis(m_pstAxisLimits->stPos.dUpper, abForced);
}

BOOL	
CAxis::MoveNegLimit(BOOL abForced)
{
	return MoveAxis(m_pstAxisLimits->stPos.dLower, abForced);
}

BOOL
CAxis::SetHomePosition(INT32 anHomePos)
{
	m_nHomePosition = anHomePos;
	m_bHomeSet = TRUE;
	return TRUE;
}

BOOL
CAxis::SetHomeReference(double adHomeRef)
{
	m_dHomeReference = adHomeRef;
	m_bHomeRefSet = TRUE;
	return TRUE;
}

INT32
CAxis::GetHomePosition(	)
{
	return m_nHomePosition;
}

BOOL
CAxis::MoveAxis(double adPosition, BOOL abForce, BOOL abAbs)
{
	return FALSE;
}

BOOL
CAxis::StopAxis()
{
	return FALSE;
}

BOOL
CAxis::EmgStopAxis()
{
	return FALSE;
}

/* This sets the position upper and lower limits explicitly
*  Default is degrees
*/
BOOL
CAxis::SetPositionLimits(double adLower, double adUpper, BOOL abIsRad)
{
	double dLower = adLower; double dUpper = adUpper;
	if (FALSE == abIsRad && eAxisRevolute == GetAxisType())
	{
		dLower = ConvertDeg2Rad(dLower);
		dUpper = ConvertDeg2Rad(dUpper);
	}
	return SetLimits(m_pstAxisLimits->stPos, dLower, dUpper);
}

/* This sets opposite values as position limits */
BOOL
CAxis::SetPositionLimits(double adPosNegLimit, BOOL abIsRad)
{
	double dLimit = adPosNegLimit;
	if (FALSE == abIsRad && eAxisRevolute == GetAxisType())
	{
		dLimit = ConvertDeg2Rad(dLimit);
	}
	return SetPositionLimits(-dLimit, dLimit);
}


BOOL
CAxis::SetVelocity(double adVel)
{
	return FALSE;
}

BOOL
CAxis::SetVelocityLimits(double adVelPosNeg)
{
	return SetVelocityLimits(-adVelPosNeg, adVelPosNeg);
}

BOOL
CAxis::SetVelocityLimits(double adLower, double adUpper)
{	
	
	return SetLimits(m_pstAxisLimits->stVel, adLower, adUpper);
}

BOOL
CAxis::SetAcceleration(double adAcc)
{
	return FALSE;
}

BOOL
CAxis::SetAccelerationLimits(double adAccPosNeg)
{
	return SetAccelerationLimits(-adAccPosNeg, adAccPosNeg);
}

BOOL
CAxis::SetAccelerationLimits(double adLower, double adUpper)
{
	return SetLimits(m_pstAxisLimits->stAcc, adLower, adUpper);
}

BOOL
CAxis::SetDeceleration(double adDec)
{
	return FALSE;
}

BOOL
CAxis::SetDecelerationLimits(double adDecPosNeg)
{
	return SetDecelerationLimits(-adDecPosNeg, adDecPosNeg);
}

BOOL
CAxis::SetDecelerationLimits(double adLower, double adUpper)
{
	return SetLimits(m_pstAxisLimits->stDec, adLower, adUpper);
}

BOOL
CAxis::SetJerk(double adJerk)
{
	return FALSE;
}

BOOL
CAxis::SetJerkLimits(double adJerkPosNeg)
{
	return SetJerkLimits(-adJerkPosNeg, adJerkPosNeg);
}

BOOL
CAxis::SetJerkLimits(double adLower, double adUpper)
{
	return SetLimits(m_pstAxisLimits->stJerk, adLower, adUpper);
}

BOOL
CAxis::SetTorque(double adJerk)
{
	return FALSE;
}

BOOL
CAxis::SetTorqueLimits(double adTorquePosNeg)
{
	return SetTorqueLimits(-adTorquePosNeg, adTorquePosNeg);
}

BOOL
CAxis::SetTorqueLimits(double adLower, double adUpper)
{
	return SetLimits(m_pstAxisLimits->stTorque, adLower, adUpper);
}

BOOL
CAxis::SetTorqueConstant(double adTorqueConst)
{
	m_dTorqueConstant = adTorqueConst;
	return TRUE;
}

BOOL
CAxis::MoveVelocity(double adVel)
{
	return FALSE;
}

BOOL
CAxis::MoveTorque(double adTorque)
{
	return FALSE;
}

void
CAxis::UpdateStartRawParams(BOOL isQuartet, UINT8 AxisIndex, INT32 anPos, INT32 anVel, INT32 anTor, INT32 anAbsPos)
{
	
	m_pstAxisStartRawParams->nPos = anPos;
	m_pstAxisStartRawParams->nVel = anVel;
	m_pstAxisStartRawParams->nTor = anTor;
	
	INT32 nAbsPos = (INT32)NormalizeAbsPosition(anAbsPos, m_dAbsResolution);

	m_pstAxisStartRawParams->nAbsPos = nAbsPos;


	INT32 nIncPos = (INT32)(nAbsPos*m_dResolution/m_dAbsResolution*m_dGearRatio);
	// INT32 nIncPos = (INT32)NormalizeAbsPosition(anAbsPos, m_dAbsResolution);

	
	
	if (m_strName == "L_ShoulderPitch" || m_strName == "R_ShoulderPitch" 
		|| m_strName == "L_ShoulderRoll" || m_strName == "R_ShoulderRoll"
		|| m_strName == "L_Wrist1" || m_strName == "L_Wrist2"		
		|| m_strName == "R_Wrist1" || m_strName == "R_Wrist2"
		|| m_strName == "L_ElbowPitch" || m_strName == "R_ElbowPitch") 
	{
		m_nOffsetPos = 0 - anPos;	
	}
	else {
		m_nOffsetPos = nIncPos - anPos;	
	}
	

	// if (m_nMasterID == 0) {
		
	// 	if (m_usAxisPos == 1 || m_usAxisPos == 2 || m_usAxisPos == 5 
	// 		|| m_usAxisPos == 8 || m_usAxisPos == 9 || m_usAxisPos == 12) {
	// 		m_nOffsetPos = 0 - anPos;
	// 	}
	// 	else		
	// 		m_nOffsetPos = nIncPos - anPos;
	// 	}
	// else if (m_nMasterID == 1) {
	// 	if (m_usAxisPos == 2 || m_usAxisPos == 3
	// 		|| m_usAxisPos == 6 || m_usAxisPos == 7) {

	// 		m_nOffsetPos = 0 - anPos;			
	// 	}
	// 	else {
	// 		if (isQuartet) {
	// 			if (AxisIndex == 0) {
	// 				m_nOffsetPos = 0  - anPos;
	// 			}
	// 			else
	// 				m_nOffsetPos = nIncPos - anPos;
				
	// 		}	
	// 	}

	// }
		
	
	// m_nOffsetPos = nIncPos - anPos;

	
}

void
CAxis::UpdateCurrentParams(INT32 anPos, INT32 anVel, INT32 anTor, INT32 anAbsPos)
{
	m_pstAxisRawParams->nPos = anPos;
	m_pstAxisRawParams->nVel = anVel;
	m_pstAxisRawParams->nTor = anTor;	
	INT32 nAbsPos = (INT32)NormalizeAbsPosition(anAbsPos, m_dAbsResolution);
	m_pstAxisRawParams->nAbsPos = nAbsPos;
	
	/* Calculate position from the home or starting position, depending whether Home Position is configured
	*/
	INT32 nCurrentPos = m_pstAxisRawParams->nPos;// - GetHomePosition();
	
	// if (FALSE == IsHomeSet()) {
	// 	nCurrentPos = m_pstAxisRawParams->nPos - GetStartRawPos();
		
	// }
	
	
	
	double dPos = ConvertRes2RadMM(nCurrentPos + m_nOffsetPos);
	double dVel = ConvertRes2RadMM(m_pstAxisRawParams->nVel);
	double dTor = ConvertRes2Tor(m_pstAxisRawParams->nTor);
	double dAbsPos = ConvertRes2RadMMAbs(m_pstAxisRawParams->nAbsPos);
	

	UpdateCurrentParams(dPos, dVel, dTor, dAbsPos);

	
}

void
CAxis::UpdateCurrentParams(double adPos, double adVel, double adTor, double dAbsPos)
{
	m_pstAxisParams->dPos = adPos;
	m_pstAxisParams->dVel = adVel;
	m_pstAxisParams->dTor = adTor;
	m_pstAxisParams->dAbsPos = dAbsPos;
	
}

INT32
CAxis::GetCurrentRawPos()
{
	return m_pstAxisRawParams->nPos;
}

INT32 CAxis::GetCurrentRawAbsPos()
{
	return m_pstAxisRawParams->nAbsPos;
}


INT32
CAxis::GetCurrentRawVel()
{
	return m_pstAxisRawParams->nVel;
}

INT32
CAxis::GetCurrentRawTor()
{
	return m_pstAxisRawParams->nTor;
}

INT32
CAxis::GetStartRawPos()
{
	return m_pstAxisStartRawParams->nPos;
}

INT32
CAxis::GetStartRawVel()
{
	return m_pstAxisStartRawParams->nVel;
}

INT32
CAxis::GetStartRawTor()
{
	return m_pstAxisStartRawParams->nTor;
}

double
CAxis::GetCurrentPos()
{
	return (m_pstAxisParams->dPos);
}

double
CAxis::GetCurrentVel()
{
	return m_pstAxisParams->dVel;
}

double
CAxis::GetCurrentTor()
{
	return m_pstAxisParams->dTor; // 모터 출력단
}

double CAxis::GetCurrentTorqueOutput()
{
	return m_pstAxisParams->dTor * GetGearRatio(); 
}



ST_AXIS_LIMITS
CAxis::GetAxisLimits()
{
	return *m_pstAxisLimits;
}

ST_AXIS_PARAMS
CAxis::GetCurrentParams()
{
	return *m_pstAxisParams;
}

ST_AXIS_RAW_PARAMS
CAxis::GetCurrentRawParams()
{
	return *m_pstAxisRawParams;
}

void
CAxis::SetState(eAxisState aeState)
{
	m_pstAxisInfo->eState = aeState;
}

/* This should be called explicitly if the joint is prismatic, otherwise pre-set values are configured */
BOOL
CAxis::SetOneTurnRef(double adOneTurnRef)
{
	if (eAxisRevolute == GetAxisType())
		return FALSE;

	m_dOneTurnRef = adOneTurnRef;
	return TRUE;
}

/* Sets the alias and position of the axis from the main controller (this is necessary for EtherCAT) */
void
CAxis::SetAliasPos(UINT16 ausAlias, UINT16 ausPos)
{
	m_usAxisAlias = ausAlias;
	m_usAxisPos = ausPos;
}

/* Sets the encoder resolution with the actual resolution, encoder ratio (X1, X2, X4), and the gear ratio (if necessary)  */
void
CAxis::SetResolution(double adRes, double adAbsRes, double adGearRatio/*=1*/, double adTransRatio/*=1*/)
//CAxis::SetResolution(double adRes, double adGearRatio/*=1*/, double adEncRatio/*=1*/)
{
	
	m_dResolution = adRes;
	m_dAbsResolution = adAbsRes;
	m_dTransRatio = adTransRatio;
	m_dGearRatio = adGearRatio;

}

BOOL
CAxis::IsLimitConfigured()
{
	/*  20221228 : raim.delgado : delete IsHomeSet from the limits list.
	*  m_bHomeSet will be handled by the state machine.
	*/
	BOOL bLimits[] = {m_pstAxisLimits->stPos.bIsSet, m_pstAxisLimits->stVel.bIsSet, m_pstAxisLimits->stAcc.bIsSet,
		m_pstAxisLimits->stDec.bIsSet, m_pstAxisLimits->stJerk.bIsSet, m_pstAxisLimits->stTorque.bIsSet };

	for (int nCnt = 0; nCnt < (int)sizeof(bLimits); nCnt++)
	{
		if (FALSE == bLimits[nCnt]) return FALSE;
	}
	return TRUE;
}

BOOL
CAxis::IsMovable()
{
	/* ensure that the motor only moves when the axis is enabled and the status is not in error
	* also for the meantime, check whether all of the limits are configured
	*/
	if ((eAxisInit < GetState()) && (TRUE == IsLimitConfigured()) && (TRUE == IsEnabled()))
		return TRUE;

	return FALSE;
}

BOOL
CAxis::IsServoOn()
{
	return FALSE;
}

BOOL 
CAxis::IsAllowablePosition(double adTarget)
{
	// if (m_usAxisPos == 7) {
	// 	printf("GetCurrentPos: %lf, Low: %lf, Up: %lf\n", GetCurrentPos(), m_pstAxisLimits->stPos.dLower, m_pstAxisLimits->stPos.dUpper);
	// }
	return CheckLimits(m_pstAxisLimits->stPos, adTarget);
}

BOOL
CAxis::IsAllowableVelocity(double adTarget)
{
	return CheckLimits(m_pstAxisLimits->stVel, adTarget);
}

BOOL
CAxis::IsAllowableAcceleration(double adTarget)
{
	return CheckLimits(m_pstAxisLimits->stAcc, adTarget);
}

BOOL
CAxis::IsAllowableDeceleration(double adTarget)
{
	return CheckLimits(m_pstAxisLimits->stDec, adTarget);
}

BOOL
CAxis::IsAllowableJerk(double adTarget)
{
	return CheckLimits(m_pstAxisLimits->stJerk, adTarget);
}

BOOL
CAxis::IsAllowableTorque(double adTarget)
{
	return CheckLimits(m_pstAxisLimits->stTorque, adTarget);
}

BOOL
CAxis::IsActuator()
{
	if (eAxisSensor <= GetAxisType())
		return FALSE;

	return TRUE;
}

BOOL
CAxis::SetLimits(ST_LIMITS& astLimit, double adLower, double adUpper)
{
	astLimit.dUpper = adUpper;
	astLimit.dLower = adLower;
	astLimit.bIsSet = TRUE;
	return TRUE;
}

BOOL
CAxis::CheckLimits(ST_LIMITS& astLimit, double adTarget)
{
	if ((astLimit.dLower <= adTarget) && (astLimit.dUpper >= adTarget))
		return TRUE;

	return FALSE;
}

/* Converts radians or millimeters to encoder resolution */
INT32
CAxis::ConvertRadMM2Res(double adPos)
{
	return (INT32)((adPos / m_dOneTurnRef) / m_dTransRatio * m_dGearRatio * m_dResolution * m_nDirection);
}

/* Converts encoder resolution to either radians or millimeters */
double
CAxis::ConvertRes2RadMM(INT32 adRes)
{	
	
	return (double)(m_dOneTurnRef * (adRes / m_dResolution / m_dGearRatio * m_dTransRatio) * m_nDirection);
	
}

double
CAxis::ConvertRes2RadMMAbs(INT32 adRes)
{	
	if (m_strName == "L_Wrist1" || m_strName == "L_Wrist2"
		|| m_strName == "R_Wrist1" || m_strName == "R_Wrist2") {
		
		return (double)(2 * M_PI * adRes / m_dAbsResolution * m_nDirection);
	}
	else	
	return (double)(m_dOneTurnRef * adRes / m_dAbsResolution * m_nDirection);
	
}

/* Calculation may change accoding to the motor driver */
INT16
CAxis::ConvertTor2Res(double adTor)
{
	// INT16 res = (INT16)(ConvertTor2Cur(adTor) * 1000.0 / GetRatedCurrent()*m_nDirection);
	
	return  (INT16)adTor;
}

double
CAxis::ConvertRes2Tor(INT32 adRes)
{
	// printf("Axis::ConvertRes2Tor\n");
	// double torque = adRes * GetRatedTorque() / 1000.0 * m_nDirection;
	return (double)adRes;
}

double CAxis::ConvertCur2Tor(double dCurrent) 
{ 
	// if (m_strName == "L_HIP_YAW" 
	// 	|| m_strName == "L_HIP_ROLL"
	// 	|| m_strName == "L_Ankle1"
	// 	|| m_strName == "L_Ankel2"
	// 	|| m_strName == "R_HIP_YAW" 
	// 	|| m_strName == "R_HIP_ROLL"
	// 	|| m_strName == "R_Ankle1"
	// 	|| m_strName == "R_Ankel2")
	// {
	// 	return dCurrent * 0.09;
	// }

	// if (m_strName == "L_HIP_PITCH"
	// 	|| m_strName == "L_KNEE"
	// 	|| m_strName == "R_HIP_PITCH"
	// 	|| m_strName == "R_KNEE")
	// {
	// 	return dCurrent * 0.2;
	// }
		
	// if (m_strName == "R_Toe")
	// {
	// 	return dCurrent * 0.08;
	// }

	return dCurrent * GetRatedTorque() / GetRatedCurrent();
 }
double CAxis::ConvertTor2Cur(double dTorque) 
{
	// if (m_strName == "L_HIP_YAW" 
	// 	|| m_strName == "L_HIP_ROLL"
	// 	|| m_strName == "L_Ankle1"
	// 	|| m_strName == "L_Ankel2"
	// 	|| m_strName == "R_HIP_YAW" 
	// 	|| m_strName == "R_HIP_ROLL"
	// 	|| m_strName == "R_Ankle1"
	// 	|| m_strName == "R_Ankel2")
	// {
	// 	return dTorque / 0.09;
	// }

	// if (m_strName == "L_HIP_PITCH"
	// 	|| m_strName == "L_KNEE"
	// 	|| m_strName == "R_HIP_PITCH"
	// 	|| m_strName == "R_KNEE")
	// {
	// 	return dTorque / 0.2;
	// }
		
	// if (m_strName == "R_Toe")
	// {
	// 	return dTorque / 0.08;
	// } 

	return dTorque * GetRatedCurrent() / GetRatedTorque(); 
}


double
CAxis::GetMaxPos()
{
	return m_pstAxisLimits->stPos.dUpper;
}

double
CAxis::GetMinPos()
{
	return m_pstAxisLimits->stPos.dLower;
}

BOOL CAxis::IsMoving()
{
	if (fabs(GetCurrentVel()) > 0.0) {
		return TRUE;
	}

	return FALSE;
}

// 절대 위치 정규화 함수
int CAxis::NormalizeAbsPosition(int nAbsPos, double dResolution)
{
    // int resolution = (int)dResolution;
    // int normalizedPos = nAbsPos + resolution;
    
    // // 반올림 처리: 해상도의 절반보다 크면 빼기, 작으면 나머지 연산
    // if ((normalizedPos % resolution) > (resolution / 2)) {
    //     normalizedPos -= resolution;
    // } 
	// else {
    //     normalizedPos = (normalizedPos + resolution) % resolution;
    // }
    
    // return normalizedPos;

	int nRes = (int)dResolution;
	int nHalf = nRes/2;
	int nMod = nAbsPos % nRes;

	if (nMod < 0) {
		nMod += nRes;
	}

	if (nMod > nHalf) {
		nMod -= nRes;
	
	}

	return nMod;
}