
#ifndef __GAMEPAD_H__
#define __GAMEPAD_H__

#include "Defines.h"
#include <linux/joystick.h>
#include <fcntl.h>
#include <unistd.h>
#include <atomic>
#include <functional>

typedef std::function<void(PVOID, PVOID, PVOID, PVOID)> CALLBACK_FN;

#define GAMEPAD_MAX_AXES     8
#define GAMEPAD_MAX_BUTTONS  16


typedef enum
{
    GAMEPAD_BUTTON_X = 0,
    GAMEPAD_BUTTON_A = 1,
    GAMEPAD_BUTTON_B = 2,
    GAMEPAD_BUTTON_Y = 3,
    GAMEPAD_BUTTON_LB = 4,
    GAMEPAD_BUTTON_RB = 5,
    GAMEPAD_BUTTON_LEFT_TRIGGER = 6,  // LT 버튼 (트리거)
    GAMEPAD_BUTTON_RIGHT_TRIGGER = 7, // RT 버튼 (트리거)
    GAMEPAD_BUTTON_BACK = 8,
    GAMEPAD_BUTTON_START = 9,
    GAMEPAD_BUTTON_LEFT_STICK = 10,   // L3 버튼 (조이스틱 눌림)
    GAMEPAD_BUTTON_RIGHT_STICK = 11,  // R3 버튼 (조이스틱 눌림)
    GAMEPAD_BUTTON_COUNT = 12
} eGamepadButton;


typedef enum
{
    GAMEPAD_AXIS_LEFT_TRIGGER = 1, // Left Trigger (LT/L2) (0 ~ 32767)
    GAMEPAD_AXIS_RIGHT_TRIGGER = 0, // Right Trigger (RT/R2) (0 ~ 32767)
    GAMEPAD_AXIS_RIGHT_X = 2,     // Right Stick X (-32767 ~ 32767)
    GAMEPAD_AXIS_RIGHT_Y = 3,     // Right Stick Y (-32767 ~ 32767)
    GAMEPAD_AXIS_LEFT_X = 4,      // Left Stick X (-32767 ~ 32767)
    GAMEPAD_AXIS_LEFT_Y = 5,      // Left Stick Y (-32767 ~ 32767)
    GAMEPAD_AXIS_D_PAD_X = 6,     // D-Pad X (좌:-32767, 중앙:0, 우:32767)
    GAMEPAD_AXIS_D_PAD_Y = 7,     // D-Pad Y (위:-32767, 중앙:0, 아래:32767)
    GAMEPAD_AXIS_COUNT = 8
} eGamepadAxis;


typedef struct
{
    BOOL buttons[GAMEPAD_MAX_BUTTONS];
    INT16 axes[GAMEPAD_MAX_AXES];
    BOOL isConnected;
    UINT64 timestamp;
} ST_GAMEPAD_STATE;


typedef struct
{
    UINT8 buttonIndex;
    BOOL buttonState;
    UINT8 axisIndex;
    INT16 axisValue;
    BOOL isButtonEvent;
    BOOL isAxisEvent;
} ST_GAMEPAD_EVENT;

class CGamepad
{
public:
    CGamepad();
    ~CGamepad();

    BOOL Init(const char* devicePath = "/dev/input/js0");
    BOOL DeInit();
    
    BOOL IsConnected() const { return m_bIsConnected; }
    
    ST_GAMEPAD_STATE GetState() const;
    
    BOOL GetButton(eGamepadButton button) const;    
    INT16 GetAxis(eGamepadAxis axis) const;       
    double GetAxisNormalized(eGamepadAxis axis) const;    
    
    BOOL ReadEvent(ST_GAMEPAD_EVENT* pEvent);    
    
    void Update();    
    void UpdateGamepadData();        
    double GetLeftStickX() const;
    double GetLeftStickY() const;
    double GetRightStickX() const;
    double GetRightStickY() const;
    double GetLeftTrigger() const;
    double GetRightTrigger() const;
    void SetEnabled(BOOL bFlag) { m_bEnabled = bFlag; }
    BOOL GetEnabled(void) { return m_bEnabled; }
    
    BOOL IsButtonPressed(eGamepadButton button) const;
    BOOL IsButtonReleased(eGamepadButton button) const;
    
    
    void PrintDebugInfo() const;
    void EnableDebugLog(BOOL bEnable) { m_bDebugLog = bEnable; }
    BOOL IsDebugLogEnabled() const { return m_bDebugLog; }

private:
    INT32				m_nJoystickFd;
    TSTRING				m_strDevicePath;
    BOOL            	m_bIsConnected;
    BOOL                m_bEnabled;
    
    
    // Mutex 제거: 쓰기는 단일 스레드(proc_gamepad_control)에서만 수행되고,
    // 개별 필드(INT16, BOOL)는 원자적 연산이므로 동기화 불필요
    ST_GAMEPAD_STATE	m_stState;
    ST_GAMEPAD_STATE	m_stPrevState; 
    
    
    CALLBACK_FN	m_pCallbackButton;
    CALLBACK_FN	m_pCallbackAxis;
    CALLBACK_FN	m_pCallbackStateChange;
    
    
    static constexpr double DEFAULT_DEADZONE = 0.2;
    double m_dDeadzone;
    
    
    BOOL m_bDebugLog;
    UINT32 m_nDebugPrintCounter;
    static constexpr UINT32 DEBUG_PRINT_INTERVAL = 100;
    
    
    BOOL OpenDevice();
    void CloseDevice();
    void ProcessEvent(const struct js_event& event);
    void UpdateState();
    UINT64 GetTimestamp();
    
    
    void OnButtonEvent(PVOID apGamepad, PVOID apEvent, PVOID apPlaceholder2, PVOID apPlaceholder3);
    void OnAxisEvent(PVOID apGamepad, PVOID apEvent, PVOID apPlaceholder2, PVOID apPlaceholder3);
    void OnStateChangeEvent(PVOID apGamepad, PVOID apState, PVOID apPlaceholder2, PVOID apPlaceholder3);
    
    
    void RegisterCallbackButton(CALLBACK_FN afnCallback);
    void RegisterCallbackAxis(CALLBACK_FN afnCallback);
    void RegisterCallbackStateChange(CALLBACK_FN afnCallback);
};

#endif // __GAMEPAD_H__

