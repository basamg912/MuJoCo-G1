#ifndef __STATE_DEF_H__
#define __STATE_DEF_H__

#define NUM_OF_JOINTS 73

// typedef enum {
//     IDLE = 0,
//     INIT,
//     RUN,
//     STOP,
// } eInterfaceState;

typedef enum {
    STATE_IDLE = 0x01,
    STATE_HOMING,
    STATE_READY,
    STATE_DOING,
    STATE_SLOWSTOP,
    STATE_FASTSTOP,
    STATE_EMERGENCY,
    STATE_ERROR,
    STATE_RESET
} eControlState;

typedef enum {
    CMD_IDLE = 0x01,
    CMD_READY,
    CMD_SLOWSTOP,
    CMD_FASTSTOP,
    CMD_EMG,
    CMD_DOING,
    CMD_HOMING,    
    CMD_RESET,
} eCmdState;

typedef enum {
	IDLE = 0,
    INIT,
    OFFSET,
	HOMING,
    RL,
	CONTROL = 5,    
	STANDSTILL,        
	STOP,
    HAND_CONTROL
	
} E_CONTROL_STATE;

typedef enum {
    L_HIP_YAW = 0,
    L_HIP_ROLL,
    L_HIP_PITCH,
    L_KNEE,
    L_ANKLE1,
    L_ANKLE2 = 5,
    L_TOE,
    R_HIP_YAW,
    R_HIP_ROLL,
    R_HIP_PITCH,
    R_KNEE = 10,
    R_ANKLE1,
    R_ANKLE2,
    R_TOE,
    WAIST_ROLL,
    WAIST_PITCH_YAW_1 = 15,
    WAIST_PITCH_YAW_2,
    NECK_PITCH,
    NECK_YAW,
    L_SHOULDER_PITCH,
    L_SHOULDER_ROLL = 20,
    L_SHOULDER_YAW,
    L_ELBOW_PITCH,
    L_WRIST1,
    L_WRIST2,
    L_WRIST_YAW = 25,
    R_SHOULDER_PITCH,
    R_SHOULDER_ROLL,
    R_SHOULDER_YAW,
    R_ELBOW_PITCH,
    R_WRIST1 = 30,
    R_WRIST2,
    R_WRIST_YAW
} E_AXIS_NAME;

#endif