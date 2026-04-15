#include "Hand.h"
#include <functional>


CHand::CHand(INT32 nNoAxes, INT32 nNoJoints, BOOL abEnabled, BOOL abConnected)     
{
    m_bConnected = abConnected;

	if (FALSE == m_bConnected) 
        SetEnabled(FALSE);

    m_pEcMaster = NULL;
    m_pstJointLimits = NULL;

    m_nNoAxes = nNoAxes;
    m_nTotalJoints = nNoJoints;
    m_pstJointLimits = new ST_JOINT_LIMITS[m_nTotalJoints];
    m_pstHandJointInfo = new ST_HAND_JOINT_INFO[m_nTotalJoints];
    m_pstRawHandJointInfo = new ST_RAW_HAND_JOINT_INFO[m_nTotalJoints];
    
    
}


CHand::~CHand()
{
    if (NULL != m_pstJointLimits)
    {
        delete[] m_pstJointLimits;
        m_pstJointLimits = NULL;
    }

    if (NULL != m_pstHandJointInfo)
    {
        delete[] m_pstHandJointInfo;
        m_pstHandJointInfo = NULL;
    }

    if (NULL != m_pstRawHandJointInfo)
    {
        delete[] m_pstRawHandJointInfo;
        m_pstRawHandJointInfo = NULL;
    }
}

BOOL CHand::Init(CEcatMaster& apEcmaster)
{
    m_pEcMaster = &apEcmaster;
    m_pEcMaster->AddSlave(&m_cEcSlave);

    m_cEcSlave.RegisterCallbackParamUpdate(std::bind(&CHand::OnSlaveUpdateRawParam, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

    return TRUE;
}


BOOL CHand::SetTargetPos(int nJoint, double dTarget)
{
    INT32 nAxisIndex = GetAxisIndex(nJoint);
    if(nAxisIndex < 0) return FALSE;
    
    m_cEcSlave.SetTargetPos(nAxisIndex, RAD2USR(dTarget));
    return TRUE;
}

double CHand::GetTargetPos(int nJoint)
{
    INT32 nAxisIndex = GetAxisIndex(nJoint);
    if(nAxisIndex < 0) return 0.0;

    double dTarget = m_cEcSlave.GetTargetPosition(nAxisIndex);

    return USR2RAD(dTarget);
}

INT16 CHand::GetRawPosition(int nJoint)
{
    INT32 nAxisIndex = GetAxisIndex(nJoint);
    if(nAxisIndex < 0) return 0.0;

    INT16 nPos = m_cEcSlave.GetPosition(nAxisIndex);

    return nPos;
}

double CHand::GetPosition(int nJoint)
{
    return m_pstHandJointInfo[nJoint].dPos;
}

INT16 CHand::GetRawCurrent(int nJoint)
{
    INT32 nAxisIndex = GetAxisIndex(nJoint);
    if(nAxisIndex < 0) return 0.0;

    INT16 nCurrent = m_cEcSlave.GetCurrent(nAxisIndex);

    return nCurrent;
}

double CHand::GetCurrent(int nJoint)
{
    return m_pstHandJointInfo[nJoint].dCurrent;
}

void CHand::SetPositionLimits(INT32 nJoint, double dPosLimitL, double dPosLimitU, BOOL abSet)
{
    m_pstJointLimits[nJoint].stPos.dLower = ConvertDeg2Rad(dPosLimitL);
    m_pstJointLimits[nJoint].stPos.dUpper = ConvertDeg2Rad(dPosLimitU);
    m_pstJointLimits[nJoint].bIsSet = abSet;
}

BOOL CHand::IsCouplingJoint(INT32 nIndex)
{
    
    for(int i=0; i<NO_COUPLING_JOINTS; ++i)
    {
        if(nIndex == COUPLING_JOINT_INDICES[i])
            return TRUE;
    }
    return FALSE;
}

void CHand::OnSlaveUpdateRawParam(PVOID apMsg, PVOID apPlaceholder0, PVOID apPlaceholder1, PVOID apPlaceholder2)
{
    ST_RAW_HAND_DATA* pstRawData = (ST_RAW_HAND_DATA*)apPlaceholder0;

    // Update raw joint info from slave
    int nAxisIdx = 0;
    for (INT32 i = 0; i < m_nTotalJoints; ++i)
    {
        if (IsCouplingJoint(i)) {
            continue;
        }            
            
        m_pstRawHandJointInfo[i].nPos = pstRawData->nPosition[nAxisIdx];
        m_pstRawHandJointInfo[i].nCurrent = pstRawData->nCurrent[nAxisIdx];

        

        nAxisIdx++;
    }

    UpdateJointInfo();
}

void CHand::UpdateJointInfo()
{
    
    // Convert raw to processed values
    for (int i = 0; i < m_nTotalJoints; i++) {        
        m_pstHandJointInfo[i].dPos = USR2RAD(m_pstRawHandJointInfo[i].nPos);
        m_pstHandJointInfo[i].dCurrent = (double)m_pstRawHandJointInfo[i].nCurrent;
    }
    
    m_pstHandJointInfo[0].dPos = m_pstHandJointInfo[1].dPos * 2;
    m_pstHandJointInfo[5].dPos = m_pstHandJointInfo[6].dPos * 2;
    m_pstHandJointInfo[9].dPos = m_pstHandJointInfo[10].dPos * 2;
    m_pstHandJointInfo[13].dPos = m_pstHandJointInfo[14].dPos * 2;
    m_pstHandJointInfo[17].dPos = m_pstHandJointInfo[18].dPos * 2;
}

INT32 CHand::GetAxisIndex(INT32 nJointIndex)
{
    if (IsCouplingJoint(nJointIndex))
    {
        return -1;
    }

    INT32 nAxisIndex = 0;
    for (int i = 0; i < nJointIndex; ++i)
    {
        if (!IsCouplingJoint(i))
        {
            nAxisIndex++;
        }
    }
    return nAxisIndex;
}

BOOL CHand::MovePosition(int nJoint, double dTargetPos)
{
    BOOL bFlag = FALSE;

    
    
    if (IsAllowablePosition(nJoint, dTargetPos)) {
        // DBG_LOG_INFO("(%s) MoveJoint: %d, TargetPos: %lf", "CHand", nJoint, ConvertRad2Deg(dTargetPos)) ;
        bFlag = SetTargetPos(nJoint, dTargetPos);
        if(!bFlag) return FALSE;
        return TRUE;
    }
    
    return FALSE;
    
}

BOOL CHand::IsAllowablePosition(int nJoint, double dPos)
{
    if (!CheckLimits(dPos, GetPositionLimits(nJoint))) {        
        DBG_LOG_WARN("(%s) Joint:%d Limited, Position:%lf.", "CHand", nJoint, ConvertRad2Deg(dPos));
        return FALSE;
    }
  
    return TRUE;
}

BOOL CHand::CheckLimits(double dTarget, ST_LIMITS stLimit)
{
    if ((stLimit.dLower <= dTarget) && (stLimit.dUpper >= dTarget))
		return TRUE;

	return FALSE;

}