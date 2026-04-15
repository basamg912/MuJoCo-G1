/*****************************************************************************
*	Name: AxisEPOS4.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header for the CAxisEPOS4 child class
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/

#ifndef __AXIS__ELMO__
#define __AXIS__ELMO__

#include "Axis.h"
#include "SlaveCIA402Base.h"
#include "SlaveElmoQuartet.h"
#include "EcatMasterBase.h"
#include "SlaveELMO.h"
#include <memory>
#include <vector>


class CAxisELMO : public CAxis
{
public:
	CAxisELMO(eAxisType aeAxisType, double adEncRes, double adAbsEncRes, double adGearRatio, double adTransRatio, 
			  BOOL abAbsoluteEncoder = TRUE, BOOL abCCW = TRUE, BOOL abEnabled = TRUE, BOOL abConnected = TRUE);
	//CAxisEPOS4(eAxisType aeAxisType, BOOL abConnected = TRUE, BOOL abEnabled = TRUE);

	CAxisELMO(eAxisType aeAxisType, double adEncRes, double adAbsEncRes, double adGearRatio, double adTransRatio, 
			  UINT8 abtAxisIndex, BOOL abAbsoluteEncoder = TRUE, BOOL abCCW = TRUE, 
			  BOOL abEnabled = TRUE, BOOL abConnected = TRUE);
	virtual ~CAxisELMO();

	// Methods for quartet support
	void SetQuartetSlave(std::shared_ptr<CElmoQuartet> apQuartetSlave);
	BOOL IsQuartetAxis() const { return m_bIsQuartet; }
	UINT8 GetQuartetAxisIndex() const { return m_btAxisIndex; }
	
	// Factory method for creating quartet axes
	static std::vector<std::unique_ptr<CAxisELMO>> CreateQuartetAxes(
		eAxisType aeAxisType, double adEncRes, double adGearRatio, double adTransRatio,
		BOOL abAbsoluteEncoder = TRUE, BOOL abCCW = TRUE, 
		BOOL abEnabled = TRUE, BOOL abConnected = TRUE);

public:
	virtual	BOOL	Init				(CEcatMaster&, INT8 abtDriveMode = CIA402_CYCLIC_TORQUE); // we overload the Init method to accomodate registration of the EtherCAT Master
	virtual	BOOL	DeInit();
	virtual BOOL	DoHoming			(	);
	virtual BOOL	MoveAxis			(double, BOOL abForce=TRUE, BOOL abAbs=TRUE);
	virtual BOOL	MoveHome			(BOOL abForce = FALSE);
	virtual BOOL	StopAxis			(	);
	virtual BOOL	EmgStopAxis			(	);
	virtual BOOL	MoveVelocity		(double);
	virtual BOOL	MoveTorque			(double); // parameter::출력단[Nm]
	virtual BOOL	ServoOn				(INT8 abtDriveMode = CIA402_CYCLIC_TORQUE);
	virtual BOOL	ChangeDriveMode		(INT8 abtDriveMode);
	virtual BOOL	ServoOff			(	);
	virtual BOOL	IsServoOn			(	);
	virtual BOOL	IsConnected			(	) { return m_bConnected; }
	
	virtual BOOL	SetVelocity			(double);
	virtual BOOL	SetAcceleration		(double);
	virtual BOOL	SetDeceleration		(double);
	virtual void	PrintAxisInfo		(	){;}

	/* todo:	1. add acquisition of Target Velocity and Torque
	*/
	virtual INT32	GetTargetRawPos		(	);
	virtual INT32	GetTargetRawVel		(	);
	virtual INT32	GetTargetRawTor		(	);
	virtual INT32	GetVelocity			(	);
	virtual INT32	GetAcceleration		(	);
	virtual INT32	GetDeceleration		(	);
	virtual INT32	GetMaxVel			(	);
	virtual INT32	GetMaxAcc			(	);
	virtual INT32	GetQuickStopDec		(	);
	virtual UINT8	GetDriveMode		(	);

	double GetAdditionalPos() override;

	virtual UINT16	GetStatusWord		(	);
	virtual UINT16	GetControlWord		(	);
	
	
	virtual void	SetVendorInfo		(UINT32 auVendorID, UINT32 auProductCode);
	virtual void	SetDCInfo			(BOOL abDCSupported, UINT16 ausActivateWord, INT32 anShiftTime);
	virtual void	SetHomingFlag		(BOOL abFlag = TRUE);


    INT16			ConvertTor2Res			(double) override;
	double			ConvertRes2Tor			(INT32) override;
	

	virtual double  GetPositionLimitLower() { return m_pstAxisLimits->stPos.dLower; }
	virtual double  GetPositionLimitUpper() { return m_pstAxisLimits->stPos.dUpper; }

	BOOL CheckAxisLimits();
	
	void UpdateHomeOffset();
	
	
	

protected:
	virtual void	OnSlaveStatusChanged	(PVOID, PVOID, PVOID, PVOID);
	virtual void	OnSlaveUpdateRawParam	(PVOID, PVOID, PVOID, PVOID);
	virtual void	SetState				(eAxisState aeState);

protected:
	CSlaveELMO		m_cEcSlave;
	std::shared_ptr<CElmoQuartet> m_pQuartetSlave;
	CEcatMaster*	m_pEcMaster;
	BOOL			m_bConnected;
	INT32			m_nPosBeforeDisconnect;

	// Device type management
	BOOL							m_bIsQuartet;		// TRUE if this axis uses quartet slave
	UINT8							m_btAxisIndex;		// 0-3 for quartet axes (0 for single)

	// Helper methods for device type abstraction
	BOOL							RouteServoOn		(INT8 abtDriveMode);
	BOOL							RouteServoOff		();
	BOOL							RouteMoveTorque		(double adTorque);
	BOOL							RouteMovePosition	(double adPosition, BOOL abAbs);
	BOOL							RouteMoveVelocity	(double adVelocity);
	BOOL							RouteStopAxis		();
	UINT16							RouteGetStatusWord	();
	INT32							RouteGetCurrentPos	();
	INT32							RouteGetCurrentVel	();
	INT32							RouteGetCurrentTor	();
	
	// Additional routing methods for getters
	UINT8							RouteGetActualDriveMode();
	BOOL							RouteSetTargetPos	(INT32 anTargetPos, BOOL abForced = FALSE, BOOL abRelative = FALSE);
	double							RouteGetAdditionalPos();
	INT32							RouteGetTargetPos	();
	INT32							RouteGetActualProfileAcc();
	INT32							RouteGetActualProfileDec();
	INT32							RouteGetTargetVel	();
	INT16							RouteGetTargetTor	();
	UINT32							RouteGetActualProfileVel();
	UINT32							RouteGetActualMaxProfileVel();
	UINT32							RouteGetActualMaxProfileAcc();
	UINT32							RouteGetActualQuickStopDec();
	UINT16							RouteGetControlWord	();


	// Device type management and routing methods
	// void							SetQuartetSlave		(std::shared_ptr<CElmoQuartet> apQuartetSlave);
	void							UpdateAxisParameters();  // Manual parameter update for both device types




}; // 


#endif // 