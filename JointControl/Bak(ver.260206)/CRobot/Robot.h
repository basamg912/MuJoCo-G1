/*****************************************************************************
*	Name: Robot.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the CRobot class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/

#ifndef __ROBOT__
#define __ROBOT__

#include "Defines.h"
#include "Axis.h"
#include "ConfigRobot.h"
#include "posix_rt.h"
#include "EcatMasterBase.h"
#include "ExtInterface.h"
#include "Hand.h"
#include "IMU.h"
#include "Gamepad.h"

/*
* we need to forward define External Interface here,
* CRobot is also forward defined in ExtInterface.h
*/
class CExtInterface;


typedef POSIX_TASK PTASK;

typedef enum
{
	eRobotInit = 0x00,
	eRobotIdle,
	eRobotOperation,
	eRobotStopped,
	eRobotEmg = 0x80,
	eRobotError,
	eRobotDeInit,
	eRobotUnknown,
}eRobotStateMach;

class CRobot
{

public:
	CRobot();
	virtual ~CRobot();
	virtual BOOL	Init(BOOL abSim);
	virtual BOOL	InitExtInterface();
	virtual BOOL	DeInit();
	virtual BOOL	IsEcatEnabled();
	virtual INT		GetTotalEcatSlaves();
	virtual INT		GetTotalRTTasks() { return m_nTotalRTTasks; }

	virtual BOOL	CheckStopTask() { return m_bStopTask; }
	virtual void	StopTasks(BOOL abStop = TRUE) { m_bStopTask = abStop; }

	virtual void	SetAgingParams(INT anDuration, double dFrequency = 1.);
	virtual void	EnableAgingTest(BOOL abEnable);
	
	BOOL	AddAxis	(CAxis*);
	BOOL	AddTaskFunction(PTASKFCN);
	BOOL 	AddHand(CHand* apHand);
	BOOL		IsSim			(	)	{ return m_bIsSim; }
	TSTRING		GetName			(	)	{ return m_strName; }
	INT			GetTotalAxis	(	);
	BOOL		GetState		(	)	{ return m_eRobotState; }
	void		SetState		(eRobotStateMach aeState) { m_eRobotState = aeState; }
	INT 		GetTotalHands() { return (INT)m_vHand.size(); }
	CGamepad*	GetGamepad() { return m_pcGamepad; }
	

private:
	eRobotStateMach m_eRobotState;

protected:
	virtual BOOL	InitEtherCAT() { return FALSE; }
	virtual BOOL	InitRTTasks();
	virtual BOOL	IsAgingTest() { return m_bAging; }
	
	virtual void	UpdateExtInterfaceData		(	);
	virtual void	OnRecvAxisCommand			(PVOID, PVOID, PVOID, PVOID);
	virtual void	DoAgingTest() { return; }
	
protected:	
	CEcatMaster* m_pcEcatMaster[NUM_OF_MASTER]; // Array of masters (can be used as single pointer in base class)
	CConfigRobot* m_pcConfigRobot;
	CExtInterface* m_pcExtInterface;

	// Gamepad
	CGamepad *m_pcGamepad = NULL;

	std::vector<CAxis*> m_vAxis;	
	std::vector<PTASK> m_vTask;
	std::vector<PTASKFCN> m_vTaskFunctions;
	
	/* TODO: abstract hand (end of arm tool) later */
	std::vector<CHand*> m_vHand;

	INT			m_nTotalRTTasks;
	BOOL		m_bStopTask;

	BOOL 		m_bAllServoOn;

	/* Aging Test using Sine Wave */
	BOOL		m_bAging;
	INT			m_nAgingDur;
	double		m_dAgingFreq;

	BOOL m_bIsSim;
	TSTRING m_strName;

	CImu m_cIMU;

	VEC_AXIS_CMD	m_vstAxisCmds;
	// std::deque<VEC_AXIS_CMD>	m_qAxisCmd;

	// allow CExtInterface to access privates
	friend class CExtInterface;
}; //CRobot




#endif // __ROBOT__