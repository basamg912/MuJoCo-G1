
#include "Gamepad.h"
#include "Defines.h"
#include <sys/time.h>
#include <sys/ioctl.h>
#include <string.h>
#include <fcntl.h>
#include <cmath>
#include <cstdlib>
#include <cerrno>

CGamepad::CGamepad()
    : m_nJoystickFd(-1)
    , m_strDevicePath("/dev/input/js0")
    , m_bIsConnected(FALSE)
    , m_pCallbackButton(NULL)
    , m_pCallbackAxis(NULL)
    , m_pCallbackStateChange(NULL)
    , m_dDeadzone(DEFAULT_DEADZONE)
    , m_bDebugLog(FALSE)
    , m_nDebugPrintCounter(0)
{
    SetEnabled(FALSE);
    memset(&m_stState, 0, sizeof(ST_GAMEPAD_STATE));    
    memset(&m_stPrevState, 0, sizeof(ST_GAMEPAD_STATE));
    for (int i = 0; i < GAMEPAD_MAX_BUTTONS; i++)
    {
        m_stState.buttons[i] = FALSE;
        m_stPrevState.buttons[i] = FALSE;
    }
    for (int i = 0; i < GAMEPAD_MAX_AXES; i++)
    {
        m_stState.axes[i] = 0;
        m_stPrevState.axes[i] = 0;
    }
    m_stState.isConnected = FALSE;
    m_stState.timestamp = 0;

    EnableDebugLog(FALSE);
}

CGamepad::~CGamepad()
{
    DeInit();
}

BOOL CGamepad::Init(const char* devicePath)
{
    if (devicePath != NULL)
    {
        m_strDevicePath = TSTRING(devicePath);
    }
    
    RegisterCallbackButton(std::bind(&CGamepad::OnButtonEvent, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    RegisterCallbackAxis(std::bind(&CGamepad::OnAxisEvent, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    RegisterCallbackStateChange(std::bind(&CGamepad::OnStateChangeEvent, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    
    if (!GetEnabled())
        return FALSE;
        
    if (OpenDevice())
    {
        m_bIsConnected = TRUE;
        m_stState.isConnected = TRUE;
        DBG_LOG_INFO("Gamepad initialized: %s", m_strDevicePath.c_str());
        
        if (m_pCallbackStateChange != NULL)
        {
            m_pCallbackStateChange(this, &m_stState, NULL, NULL);
        }
        return TRUE;
    }
    
    return FALSE;
}

BOOL CGamepad::DeInit()
{
    CloseDevice();
    m_bIsConnected = FALSE;
    m_stState.isConnected = FALSE;
    return TRUE;
}

BOOL CGamepad::OpenDevice()
{
    if (m_nJoystickFd >= 0)
    {
        CloseDevice();
    }
    
    m_nJoystickFd = open(m_strDevicePath.c_str(), O_RDONLY | O_NONBLOCK);
    if (m_nJoystickFd < 0)
    {
        DBG_LOG_WARN("Cannot open gamepad device: %s (errno: %d)", m_strDevicePath.c_str(), errno);
        return FALSE;
    }
    
    char name[256];
    UINT8 axes = 0, buttons = 0;
    
    if (ioctl(m_nJoystickFd, JSIOCGNAME(sizeof(name)), name) >= 0)
    {
        DBG_LOG_INFO("Gamepad name: %s", name);
    }
    
    ioctl(m_nJoystickFd, JSIOCGAXES, &axes);
    ioctl(m_nJoystickFd, JSIOCGBUTTONS, &buttons);
    
    DBG_LOG_INFO("Gamepad initialized: %s", name);
    DBG_LOG_INFO("Gamepad axes: %d, buttons: %d", axes, buttons);
    
    if (strstr(name, "RumblePad") != NULL || strstr(name, "F710") != NULL)
    {
        DBG_LOG_INFO("Gamepad mode: DirectInput (recommended for Linux)");
    }
    
    {
        struct js_event event;
    
        INT32 nInitEventsRead = 0;
        const INT32 MAX_INIT_EVENTS = 100;
        
        while (nInitEventsRead < MAX_INIT_EVENTS)
        {
            ssize_t bytes = read(m_nJoystickFd, &event, sizeof(event));
            if (bytes <= 0)
            {
                
                break;
            }
            
            if (bytes == sizeof(event))
            {
                if (event.type & JS_EVENT_AXIS)
                {
                    if (event.number >= 0 && event.number < GAMEPAD_MAX_AXES)
                    {
                        m_stState.axes[event.number] = event.value;
                        nInitEventsRead++;
                    }
                }
                else if (event.type & JS_EVENT_BUTTON)
                {
                    if (event.number >= 0 && event.number < GAMEPAD_MAX_BUTTONS)
                    {
                        m_stState.buttons[event.number] = event.value ? TRUE : FALSE;
                        nInitEventsRead++;
                    }
                }
            }
        }
        
        if (m_bDebugLog && nInitEventsRead > 0)
        {
            DBG_LOG_INFO("[Gamepad] Read %d initial state events", nInitEventsRead);
            
            TSTRING axisValues = "[Gamepad] Initial axes: ";
            for (int i = 0; i < GAMEPAD_MAX_AXES && i < axes; i++)
            {
                char buf[32];
                snprintf(buf, sizeof(buf), "AXIS%d=%d ", i, m_stState.axes[i]);
                axisValues += buf;
            }
            DBG_LOG_INFO("%s", axisValues.c_str());
        }
    }
    
    return TRUE;
}

void CGamepad::CloseDevice()
{
    if (m_nJoystickFd >= 0)
    {
        printf("Close device\n");
        close(m_nJoystickFd);
        m_nJoystickFd = -1;
    }
}

ST_GAMEPAD_STATE CGamepad::GetState() const
{
    
    ST_GAMEPAD_STATE state;
    state.isConnected = m_stState.isConnected;
    state.timestamp = m_stState.timestamp;
    
    
    for (int i = 0; i < GAMEPAD_MAX_BUTTONS; i++)
    {
        state.buttons[i] = m_stState.buttons[i];
    }
    for (int i = 0; i < GAMEPAD_MAX_AXES; i++)
    {
        state.axes[i] = m_stState.axes[i];
    }
    
    return state;
}

BOOL CGamepad::GetButton(eGamepadButton button) const
{
    
    if (button < 0 || button >= GAMEPAD_BUTTON_COUNT)
        return FALSE;
    
    return m_stState.buttons[button];
}

INT16 CGamepad::GetAxis(eGamepadAxis axis) const
{
    if (axis < 0 || axis >= GAMEPAD_AXIS_COUNT)
        return 0;
    
    return m_stState.axes[axis];
}

double CGamepad::GetAxisNormalized(eGamepadAxis axis) const
{
    INT16 rawValue = GetAxis(axis);
    
    return (double)rawValue / 32767.0;
}

BOOL CGamepad::ReadEvent(ST_GAMEPAD_EVENT* pEvent)
{
    if (pEvent == NULL || m_nJoystickFd < 0)
        return FALSE;
    
    struct js_event event;
    ssize_t bytes = read(m_nJoystickFd, &event, sizeof(event));
    
    if (bytes == 0)
    {
        DBG_LOG_WARN("[Gamepad] EOF detected, closing device");
        CloseDevice();
        m_bIsConnected = FALSE;
        m_stState.isConnected = FALSE;
        return FALSE;
    }
    
    if (bytes < 0)
    {
        
        int saved_errno = errno;
        
        
        if (saved_errno == EAGAIN || saved_errno == EWOULDBLOCK)
        {
           
            return FALSE;
        }
        else if (saved_errno == ENODEV || saved_errno == EIO || saved_errno == EBADF)
        {
           
            DBG_LOG_WARN("[Gamepad] Device error (errno: %d), closing device", saved_errno);
            CloseDevice();
            m_bIsConnected = FALSE;
            m_stState.isConnected = FALSE;
            return FALSE;
        }
        
        return FALSE;
    }
    
    if (bytes != sizeof(event))
    {
        
        return FALSE;
    }
    
    
    pEvent->isButtonEvent = FALSE;
    pEvent->isAxisEvent = FALSE;
    
    if (event.type & JS_EVENT_BUTTON)
    {
        pEvent->isButtonEvent = TRUE;
        pEvent->buttonIndex = event.number;
        pEvent->buttonState = event.value ? TRUE : FALSE;
    }
    else if (event.type & JS_EVENT_AXIS)
    {
        pEvent->isAxisEvent = TRUE;
        pEvent->axisIndex = event.number;
        pEvent->axisValue = event.value;
    }
    
    ProcessEvent(event);
    return TRUE;
}

void CGamepad::ProcessEvent(const struct js_event& event)
{
    
    BOOL bCallButtonCallback = FALSE;
    BOOL bCallAxisCallback = FALSE;
    ST_GAMEPAD_EVENT btnEvent;
    ST_GAMEPAD_EVENT axisEvent;
    CALLBACK_FN pButtonCallback = NULL;
    CALLBACK_FN pAxisCallback = NULL;
    
    {
        static BOOL bFirstEvent = TRUE;
        if (bFirstEvent)
        {
            // m_stPrevState 초기화 (개별 필드 복사)
            m_stPrevState.isConnected = m_stState.isConnected;
            m_stPrevState.timestamp = m_stState.timestamp;
            for (int i = 0; i < GAMEPAD_MAX_BUTTONS; i++)
            {
                m_stPrevState.buttons[i] = m_stState.buttons[i];
            }
            for (int i = 0; i < GAMEPAD_MAX_AXES; i++)
            {
                m_stPrevState.axes[i] = m_stState.axes[i];
            }
            bFirstEvent = FALSE;
        }
        
        if (event.type & JS_EVENT_BUTTON)
        {
            
            if (event.number >= 0 && event.number < GAMEPAD_MAX_BUTTONS)
            {
                BOOL oldState = m_stState.buttons[event.number];
                m_stState.buttons[event.number] = event.value ? TRUE : FALSE;
                
              
                if (m_bDebugLog && oldState != m_stState.buttons[event.number])
                {
                    const char* btnName = "?";
                    if (event.number == GAMEPAD_BUTTON_X) btnName = "X";
                    else if (event.number == GAMEPAD_BUTTON_A) btnName = "A";
                    else if (event.number == GAMEPAD_BUTTON_B) btnName = "B";
                    else if (event.number == GAMEPAD_BUTTON_Y) btnName = "Y";
                    else if (event.number == GAMEPAD_BUTTON_LB) btnName = "LB";
                    else if (event.number == GAMEPAD_BUTTON_RB) btnName = "RB";
                    else if (event.number == GAMEPAD_BUTTON_LEFT_TRIGGER) btnName = "LT";
                    else if (event.number == GAMEPAD_BUTTON_RIGHT_TRIGGER) btnName = "RT";
                    else if (event.number == GAMEPAD_BUTTON_BACK) btnName = "Back";
                    else if (event.number == GAMEPAD_BUTTON_START) btnName = "Start";
                    else if (event.number == GAMEPAD_BUTTON_LEFT_STICK) btnName = "L3";
                    else if (event.number == GAMEPAD_BUTTON_RIGHT_STICK) btnName = "R3";
                    
                    DBG_LOG_INFO("[Gamepad] Button %d (%s): %s", event.number, btnName,
                               m_stState.buttons[event.number] ? "PRESSED" : "RELEASED");
                }
                
                if (m_pCallbackButton != NULL && oldState != m_stState.buttons[event.number])
                {
                    bCallButtonCallback = TRUE;
                    pButtonCallback = m_pCallbackButton;
                    btnEvent.isButtonEvent = TRUE;
                    btnEvent.buttonIndex = event.number;
                    btnEvent.buttonState = m_stState.buttons[event.number];
                    btnEvent.isAxisEvent = FALSE;
                }
            }
        }
        else if (event.type & JS_EVENT_AXIS)
        {
            if (event.number >= 0 && event.number < GAMEPAD_MAX_AXES)
            {
                INT16 oldValue = m_stState.axes[event.number];
                m_stState.axes[event.number] = event.value;
                
                
                const INT16 DEADZONE = 1000;
                INT16 diff = m_stState.axes[event.number] - oldValue;
                if (diff < 0) diff = -diff;
                
                if (m_bDebugLog)
                {
                
                    const char* axisNames[] = {"LX", "LY", "LT", "RX", "RY", "RT", "DX", "DY"};
                    const char* axisName = (event.number >= 0 && event.number < 8) ? axisNames[event.number] : "?";
                    
                
                    if (diff > 0)
                    {
                        DBG_LOG_INFO("[Gamepad] Axis %d (%s): raw=%d norm=%.3f (diff=%d)", 
                                   event.number, axisName, event.value, 
                                   (double)event.value / 32767.0, diff);
                    }
                    else
                    {
                        
                        DBG_LOG_INFO("[Gamepad] Axis %d (%s): raw=%d norm=%.3f (init)", 
                                   event.number, axisName, event.value, 
                                   (double)event.value / 32767.0);
                    }
                }
                if (m_pCallbackAxis != NULL && diff > DEADZONE)
                {
                    bCallAxisCallback = TRUE;
                    pAxisCallback = m_pCallbackAxis; 
                    axisEvent.isAxisEvent = TRUE;
                    axisEvent.axisIndex = event.number;
                    axisEvent.axisValue = event.value;
                    axisEvent.isButtonEvent = FALSE;
                }
            }
        }
        
        m_stState.timestamp = GetTimestamp();
    } 
    
    if (bCallButtonCallback && pButtonCallback != NULL)
    {
        try
        {
            pButtonCallback(this, &btnEvent, NULL, NULL);
        }
        catch (...)
        {
            DBG_LOG_ERROR("[Gamepad] Exception in button callback");
        }
    }
    
    if (bCallAxisCallback && pAxisCallback != NULL)
    {
        try
        {
            pAxisCallback(this, &axisEvent, NULL, NULL);
        }
        catch (...)
        {
            DBG_LOG_ERROR("[Gamepad] Exception in axis callback");
        }
    }
}

void CGamepad::Update()
{
    if (!m_bIsConnected || m_nJoystickFd < 0)
    {
        static UINT32 nReconnectAttempts = 0;
        if (m_nJoystickFd < 0)
        {
            const UINT32 RECONNECT_INTERVAL = 1000;
            
            if (nReconnectAttempts == 0 || (nReconnectAttempts % RECONNECT_INTERVAL == 0))
            {
                
                if (OpenDevice())
                {
                    m_bIsConnected = TRUE;
                    m_stState.isConnected = TRUE;
                    
                    if (m_pCallbackStateChange != NULL)
                    {
                        
                        m_pCallbackStateChange(this, &m_stState, NULL, NULL);
                    }
                    nReconnectAttempts = 0; 
                }
            }
            nReconnectAttempts++;
        }
        return;
    }
    
    ST_GAMEPAD_EVENT event;
    ReadEvent(&event); // 한 번만 읽기 (RT 루프 부담 최소화, 다음 프레임에서 처리)
    
}

void CGamepad::RegisterCallbackButton(CALLBACK_FN afnCallback)
{
    m_pCallbackButton = std::move(afnCallback);
}

void CGamepad::RegisterCallbackAxis(CALLBACK_FN afnCallback)
{
    m_pCallbackAxis = std::move(afnCallback);
}

void CGamepad::RegisterCallbackStateChange(CALLBACK_FN afnCallback)
{
    m_pCallbackStateChange = std::move(afnCallback);
}

void CGamepad::OnButtonEvent(PVOID apGamepad, PVOID apEvent, PVOID apPlaceholder2, PVOID apPlaceholder3)
{
    if (apEvent == NULL)
        return;
    
    ST_GAMEPAD_EVENT* pEvent = (ST_GAMEPAD_EVENT*)apEvent;
    
    if (pEvent->isButtonEvent)
    {
        
        if (m_bDebugLog)
        {
            const char* btnNames[] = {"X", "A", "B", "Y", "LB", "RB", "Back", "Start", "L3", "R3"};
            const char* btnName = (pEvent->buttonIndex < 10) ? btnNames[pEvent->buttonIndex] : "?";
            DBG_LOG_INFO("[Gamepad] Button %d (%s): %s", 
                        pEvent->buttonIndex, btnName,
                        pEvent->buttonState ? "PRESSED" : "RELEASED");
        }
        
    }
}

void CGamepad::OnAxisEvent(PVOID apGamepad, PVOID apEvent, PVOID apPlaceholder2, PVOID apPlaceholder3)
{
    if (apEvent == NULL)
        return;
    
    ST_GAMEPAD_EVENT* pEvent = (ST_GAMEPAD_EVENT*)apEvent;
    
    if (pEvent->isAxisEvent)
    {
        
        if (m_bDebugLog)
        {
            const INT16 LOG_THRESHOLD = 1000;
            if (abs(pEvent->axisValue) > LOG_THRESHOLD)
            {
                const char* axisNames[] = {"LX", "LY", "LT", "RX", "RY", "RT", "DX", "DY"};
                const char* axisName = (pEvent->axisIndex < 8) ? axisNames[pEvent->axisIndex] : "?";
                DBG_LOG_INFO("[Gamepad] Axis %d (%s): raw=%d norm=%.3f", 
                           pEvent->axisIndex, axisName, pEvent->axisValue,
                           (double)pEvent->axisValue / 32767.0);
            }
        }
        
    }
}

void CGamepad::OnStateChangeEvent(PVOID apGamepad, PVOID apState, PVOID apPlaceholder2, PVOID apPlaceholder3)
{
    if (apState == NULL)
        return;
    
    ST_GAMEPAD_STATE* pState = (ST_GAMEPAD_STATE*)apState;
    
    if (pState->isConnected)
    {
        DBG_LOG_INFO("[Gamepad] State changed: Connected to %s", m_strDevicePath.c_str());
    }
    else
    {
        DBG_LOG_WARN("[Gamepad] State changed: Disconnected");
    }
}

UINT64 CGamepad::GetTimestamp()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (UINT64)(ts.tv_sec * 1000000000LL + ts.tv_nsec);
}

void CGamepad::UpdateGamepadData()
{
    
    Update();
    
    {
        // m_stPrevState 업데이트 (개별 필드 복사로 안전성 보장)
        m_stPrevState.isConnected = m_stState.isConnected;
        m_stPrevState.timestamp = m_stState.timestamp;
        for (int i = 0; i < GAMEPAD_MAX_BUTTONS; i++)
        {
            m_stPrevState.buttons[i] = m_stState.buttons[i];
        }
        for (int i = 0; i < GAMEPAD_MAX_AXES; i++)
        {
            m_stPrevState.axes[i] = m_stState.axes[i];
        }
    }
    
    if (m_bDebugLog)
    {
        m_nDebugPrintCounter++;
        if (m_nDebugPrintCounter >= DEBUG_PRINT_INTERVAL)
        {
            
            if (m_bIsConnected)
            {
                
                UINT32 nButtonCount = 0;
                for (int i = 0; i < GAMEPAD_BUTTON_COUNT && i < 4; i++)
                {
                    if (m_stState.buttons[i]) nButtonCount++;
                }
                
                DBG_LOG_INFO("[Gamepad] Connected - Buttons: %u active", nButtonCount);
            }
            else
            {
                DBG_LOG_WARN("[Gamepad] Not connected");
            }
            m_nDebugPrintCounter = 0;
        }
    }
    
    static BOOL bFirstCall = TRUE;
    if (bFirstCall)
    {
        bFirstCall = FALSE;
        DBG_LOG_INFO("[Gamepad] UpdateGamepadData called - Connected: %s, DebugLog: %s", 
                     m_bIsConnected ? "YES" : "NO",
                     m_bDebugLog ? "ENABLED" : "DISABLED");
    }
}

double CGamepad::GetLeftStickX() const
{
    if (!m_bIsConnected)
        return 0.0;
        
    double value = GetAxisNormalized(GAMEPAD_AXIS_LEFT_X);
    return (fabs(value) > m_dDeadzone) ? value : 0.0;
}

double CGamepad::GetLeftStickY() const
{
    if (!m_bIsConnected)
        return 0.0;
        
    double value = GetAxisNormalized(GAMEPAD_AXIS_LEFT_Y);
    return (fabs(value) > m_dDeadzone) ? value : 0.0;
}

double CGamepad::GetRightStickX() const
{
    if (!m_bIsConnected)
        return 0.0;
        
    double value = GetAxisNormalized(GAMEPAD_AXIS_RIGHT_X);
    return (fabs(value) > m_dDeadzone) ? value : 0.0;
}

double CGamepad::GetRightStickY() const
{
    if (!m_bIsConnected)
        return 0.0;
        
    double value = GetAxisNormalized(GAMEPAD_AXIS_RIGHT_Y);
    return (fabs(value) > m_dDeadzone) ? value : 0.0;
}

double CGamepad::GetLeftTrigger() const
{
    if (!m_bIsConnected)
        return 0.0;
        
    BOOL btnPressed = GetButton(GAMEPAD_BUTTON_LEFT_TRIGGER);
    if (btnPressed)
    {
        return 1.0; 
    }
    
    double axisValue = GetAxisNormalized(GAMEPAD_AXIS_LEFT_TRIGGER);
    return (axisValue > m_dDeadzone) ? axisValue : 0.0;
}

double CGamepad::GetRightTrigger() const
{
    
    if (!m_bIsConnected)
        return 0.0;
        
    BOOL btnPressed = GetButton(GAMEPAD_BUTTON_RIGHT_TRIGGER);
    if (btnPressed)
    {
        return 1.0; 
    }
    
    double axisValue = GetAxisNormalized(GAMEPAD_AXIS_RIGHT_TRIGGER);
    return (axisValue > m_dDeadzone) ? axisValue : 0.0;
}

BOOL CGamepad::IsButtonPressed(eGamepadButton button) const
{
    // 안전한 범위 체크 (음수와 최대값 모두 확인)
    if (button < 0 || button >= GAMEPAD_BUTTON_COUNT)
        return FALSE;
    
    return (m_stState.buttons[button] && !m_stPrevState.buttons[button]);
}

BOOL CGamepad::IsButtonReleased(eGamepadButton button) const
{
    if (button < 0 || button >= GAMEPAD_BUTTON_COUNT)
        return FALSE;
    
    return (!m_stState.buttons[button] && m_stPrevState.buttons[button]);
}

void CGamepad::PrintDebugInfo() const
{
    
    if (!m_bIsConnected || !m_stState.isConnected)
    {
        DBG_LOG_WARN("[Gamepad] Status: Not connected");
        return;
    }
    
    DBG_LOG_INFO("[Gamepad] Status: Connected");
    
    // 버튼 상태 출력
    TSTRING btnStatus = "[Gamepad] Buttons: ";
    const char* btnNames[] = {"X", "A", "B", "Y", "LB", "RB", "Back", "Start", "L3", "R3"};
    
    for (int i = 0; i < GAMEPAD_BUTTON_COUNT; i++)
    {
        if (m_stState.buttons[i])
        {
            btnStatus += btnNames[i];
            btnStatus += " ";
        }
    }
    
    if (btnStatus.length() == 18) 
    {
        btnStatus += "None";
    }
    
    DBG_LOG_INFO("%s", btnStatus.c_str());
    
    TSTRING axisStatus = "[Gamepad] Axes: ";
    BOOL bAxisActive = FALSE;
    
    const char* axisNames[] = {"LX", "LY", "LT", "RX", "RY", "RT", "DX", "DY"};
    
    for (int i = 0; i < GAMEPAD_AXIS_COUNT; i++)
    {
        INT16 value = m_stState.axes[i];
        
        const INT16 DEADZONE_THRESHOLD = 1000;
        if (abs(value) > DEADZONE_THRESHOLD)
        {
            if (bAxisActive) axisStatus += ", ";
            char axisVal[32];
            sprintf(axisVal, "%s=%d", axisNames[i], value);
            axisStatus += axisVal;
            bAxisActive = TRUE;
        }
    }
    
    if (!bAxisActive)
    {
        axisStatus += "All centered";
    }
    
    DBG_LOG_INFO("%s", axisStatus.c_str());
    
    double leftX = GetAxisNormalized(GAMEPAD_AXIS_LEFT_X);
    double leftY = GetAxisNormalized(GAMEPAD_AXIS_LEFT_Y);
    double rightX = GetAxisNormalized(GAMEPAD_AXIS_RIGHT_X);
    double rightY = GetAxisNormalized(GAMEPAD_AXIS_RIGHT_Y);
    
    DBG_LOG_INFO("[Gamepad] Sticks: L(%.3f, %.3f) R(%.3f, %.3f)", 
                 leftX, leftY, rightX, rightY);
    
    double leftTrigger = GetAxisNormalized(GAMEPAD_AXIS_LEFT_TRIGGER);
    double rightTrigger = GetAxisNormalized(GAMEPAD_AXIS_RIGHT_TRIGGER);
    
    DBG_LOG_INFO("[Gamepad] Triggers: LT=%.3f RT=%.3f", leftTrigger, rightTrigger);
}

