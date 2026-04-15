
#ifndef __HUMANOID__
#define __HUMANOID__

#include "Robot.h"

#include "SlaveBeckhoffCU1124.h"
#include "ROS2Executor.h"
#include "ROS2Node.h"
#include <cstddef>
#include <list>
#include <queue>
#include <mutex>
#include <map>
#include <string>

#include "AxisELMO.h"
#include "Hand.h"

#include "ROS2PointCloud.h"

// #include "concurrentqueue.h"
#include "StateDef.h"
#include "JointControl.h"
#include "RLControl.h"
#include "MimicRLControl.h"

#define NO_JOINTS 3


typedef std::vector<double> VECDOUBLE;
typedef std::list<UINT64>   LISTULONG;
typedef std::list<INT32>	LISTINT;




typedef struct stDataLog
{
	LISTULONG	vecTimestamp;
	LISTINT		vecTarPos;
	LISTINT		vecActPos;
	LISTINT		vecActTor;

	stDataLog()
	{
		vecTimestamp.clear();
		vecTarPos.clear();
		vecActPos.clear();
		vecActTor.clear();
	}

}ST_DATALOG;

class CHumanoid : public CRobot
{
public:
	CHumanoid(CConfigRobot* apcConfig = NULL);
	virtual ~CHumanoid();

public:
	virtual BOOL	Init					(BOOL abSim);
	virtual BOOL	DeInit				
		(	);
	void	WriteDataLog();

private:
	TSTRING		m_strDataLog;
	ST_DATALOG	m_stDataLog[32];

protected:
	virtual BOOL	InitEtherCAT			(	);
	virtual BOOL	InitConfig				(	);
	virtual void	DoAgingTest				(	);

	
	CSlaveBeckhoffCU1124** m_pcTerminals;

	CAxisELMO** m_pcEcatElmo;
	CHand** m_pcHand;

	

	/* TEMPORARY */
	char	m_cKeyPress;
	void	DoInput(double time);
	void	DoHoming() {};

protected:
	friend void	proc_main_control(void*);
	friend void proc_ethercat_control(void*);
	friend void	proc_keyboard_control(void*);
	friend void	proc_logger(void*); 
	friend void proc_terminal_output(void*);
	friend void proc_learning(void*);
	friend void proc_imu(void *);
	friend void proc_gamepad_control(void*);
	

	virtual void	OnRecvROS2Msg				(PVOID, PVOID, PVOID, PVOID);
	virtual void 	OnRecvROS2PointCloud(PVOID, PVOID, PVOID, PVOID);
	virtual void	OnRecvROS2AxisInfoCmd(PVOID, PVOID, PVOID, PVOID);
	virtual void	OnRecvROS2Imu(PVOID, PVOID, PVOID, PVOID);

	

private:
	BOOL m_bEcatOP;
	UINT32	m_nEcatCycle;

	// CROS2StrSub* m_pcSubScriber;
	// CROS2Executor m_cExecutor;
	CROS2ImuSub* m_pcSubscriber_Imu;
	CROS2PointCloudSub* m_pcSubscriber_PointCloud;
	CROS2PointCloudPub* m_pcPublisher_PointCloud;	
	CROS2AxisInfoCmdSub* m_pcSubscriber_AxisInfoCmd;
	CROS2AxisInfoStatePub* m_pcPublisher_AxisInfoState;
	CROS2Executor* m_pcRos2Executor;
	
	CJointControl *m_pcJointControl = NULL;
	RLControl *m_pcRLControl = NULL;
	MimicRLControl *m_pcMimicRLControl = NULL;


	

	sensor_msgs::msg::Imu m_ImuData;
	sensor_msgs::msg::PointCloud2 m_PointCloudData;
	axis_info::msg::AxisCmd m_AxisCmd;
	axis_info::msg::AxisState m_stAxisState;
	std::queue<axis_info::msg::AxisCmd> m_qAxisCmd;
	std::mutex m_mtxAxisCmd;
	std::map<std::string, int> m_mapAxisNameToIdx;

	UINT8 m_unTask;
	

	// moodycamel::ConcurrentQueue<sensor_msgs::msg::Imu> m_ImuQueue;

	

	
	double m_dPositions[NUM_OF_JOINTS] = {0.0,};
	double m_dVelocities[NUM_OF_JOINTS] = {0.0,};
	double m_dAbsPositions[NUM_OF_JOINTS] = {0.0,};
	// double m_dDesiredPos[NO_JOINTS] = {0.0, };
	// double m_dDesiredVel[NO_JOINTS] = {0.0, };
	


	static const size_t MAX_AXIS_CMD_QUEUE_SIZE = 10;
	UINT8 m_unCmdState;

	void SetJointPDGain();

	E_CONTROL_STATE m_eControlState;

	double temp_offset[NUM_OF_JOINTS] = {0.0, };


public:
	// const sensor_msgs::msg::Imu& GetImuData() { return m_ImuData; }	
	const sensor_msgs::msg::PointCloud2& GetPointCloudData() { return m_PointCloudData; }
	const axis_info::msg::AxisCmd& GetAxisCmd() { return m_AxisCmd; }

	BOOL PopAxisCmd(axis_info::msg::AxisCmd& cmd);
	size_t GetAxisCmdCount();

	sensor_msgs::msg::Imu GetImuData();
	
	CAxisELMO* GetEcatElmo(int nAxis) { return m_pcEcatElmo[nAxis]; }

	void SetTargetTorque(double* dTarget);
	void SetHandTargetPositions(double* dTargetPos);
	void SetAxisInfoStateForROS2();

	// Hand target queue and mutex (Hyesung)
	bool m_isHandMove = FALSE;
	std::queue<VECDOUBLE> m_qRightHandTargetPos;
	std::mutex m_mtxRightHandTargetPos;
	
	// Interpolation variable (Hyesung)
	VECDOUBLE m_vecRightHandStartPos, m_vecLeftHandStartPos;
	VECDOUBLE m_vecRightHandEndPos, m_vecLeftHandEndPos;
	VECDOUBLE m_vecRightHandCurrentRefPos, m_vecLeftHandCurrentRefPos;
	int m_nRightHandInterpTick = 0;
	int m_nLeftHandInterpTick = 0;

	// Hand motion csv reader variable and function (Hyesung)
	std::vector<VECDOUBLE> m_vecRightHandMotionData;
	std::vector<VECDOUBLE> m_vecLeftHandMotionData;
    int m_nMotionPlayIndex = 0;
    
    bool LoadMotionDataCSV(const std::string& filename);

}; 
#endif //__HUMANOID__
