#include "motor.h"
#include "calibration.h"
#include "control.h"
#include "collision.h"
#include "demo.h"
#include "mission.h"
#include "reaction.h"
#include "verify.h"
#include "robot_config.h"
#include "robot_pins.h"
#include <stdint.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#if ENABLE_UART_DEBUG
#include "avoid.h"
#include "line_follow.h"
#include <stdio.h>
#endif

static volatile uint32_t gMsTicks = 0U;
/*
 * Local compile-time verification switch.
 * Set to 1 to run the front-sensor verification stage instead of normal/demo
 * control. Default remains 0 to preserve current behavior.
 */
#define APP_MODE_VERIFY 0U

/* Debug/watch variables for the calibration stage. */
static volatile uint8_t gDbgLatestLine = 0U;
static volatile uint8_t gDbgBumpState = 0U;
static volatile uint16_t gDbgLineWhite[8] = {0U};
static volatile uint16_t gDbgLineBlack[8] = {0U};
static volatile uint16_t gDbgLineThreshold[8] = {0U};
static volatile float gDbgUltraLeftBaselineCm = -1.0f;
static volatile float gDbgUltraRightBaselineCm = -1.0f;
static volatile ControlState gDbgControlState = CONTROL_STATE_LINE_FOLLOW;
static volatile uint8_t gDbgVerifyBumpMask = 0U;
static volatile int8_t gDbgVerifyBumpIndex = -1;
static volatile uint8_t gDbgVerifyMultipleBumps = 0U;
static volatile uint16_t gDbgVerifyFrontIrRaw = 0U;
static volatile uint8_t gDbgCollisionZone = COLLISION_NONE;
static volatile uint8_t gDbgDemoStage = DEMO_STAGE_STARTUP;
static volatile uint8_t gDbgDemoBumpMask = 0U;
static volatile uint8_t gDbgDemoCollisionZone = COLLISION_NONE;
static volatile uint8_t gDbgDemoLineData = 0U;
static volatile uint16_t gDbgDemoFrontIrRaw = 0U;
static volatile float gDbgDemoUltraLeftCm = -1.0f;
static volatile float gDbgDemoUltraRightCm = -1.0f;
static volatile uint8_t gDbgDemoEmergencyActive = 0U;

#if ENABLE_UART_DEBUG
static volatile uint32_t gDbgPrintLoopCount = 0U;
static volatile ControlState gDbgPrevPrintControlState = CONTROL_STATE_LINE_FOLLOW;
static volatile AvoidState gDbgPrevPrintAvoidState = AVOID_STATE_IDLE;
static volatile unsigned int gDbgPrevLineMode = LINE_FOLLOW_MODE_FOLLOW_LINE;
static volatile unsigned int gDbgPrevStartupLatched = 0U;
static volatile unsigned int gDbgPrevStartupValid = 0U;
static volatile unsigned int gDbgPrevPidFirstSample = 1U;
static volatile unsigned int gDbgPrevLineValid = 0U;
static volatile unsigned int gDbgPrevPidClamped = 0U;
static volatile unsigned int gDbgPrevMissionWatchdog = 0U;
static volatile unsigned int gDbgPrevMissionTimeout = 0U;
static volatile unsigned int gDbgPrevMissionZoneSwitches = 0U;
static volatile unsigned int gDbgPrevMissionTorquePhase = MISSION_TORQUE_PHASE_ENTRY;
static volatile unsigned int gDbgPrevMissionTorqueFault = 0U;
static volatile unsigned int gDbgPrevAvoidNearMiss = 0U;
static volatile unsigned int gDbgPrevAvoidBumpEvents = 0U;
static volatile unsigned int gDbgPrevAvoidAttempts = 0U;
static volatile unsigned int gDbgPrevAvoidSuccess = 0U;
static volatile unsigned int gDbgPrevAvoidTimeouts = 0U;
static const eUSCI_UART_ConfigV1 gDebugUartConfig = {
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,
    1U,
    10U,
    0U,
    EUSCI_A_UART_NO_PARITY,
    EUSCI_A_UART_LSB_FIRST,
    EUSCI_A_UART_ONE_STOP_BIT,
    EUSCI_A_UART_MODE,
    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,
    EUSCI_A_UART_8_BIT_LEN
};

static void DebugUart_Init(void);
static void DebugUart_WriteString(const char *text);
static const char *Debug_ControlStateName(ControlState state);
static const char *Debug_AvoidStateName(AvoidState state);
static const char *Debug_LineFollowModeName(unsigned int mode);
static const char *Debug_LineMotionName(unsigned int motion);
static const char *Debug_MissionTorquePhaseName(unsigned int phase);
static void Debug_PrintStateTransitions(void);
static void Debug_PrintLineTuningEvents(void);
static void Debug_PrintLineTuningStatus(void);
#endif

static void Clock_Init(void);
static void SysTick_Init(void);
static void DelayMs(uint32_t ms);
void SysTick_Handler(void);

int main(void)
{
    Clock_Init();
#if ENABLE_UART_DEBUG
    DebugUart_Init();
    DebugUart_WriteString("MODE DEBUG\n");
#endif

    /* Leave motors initialized and idle so the robot stays in a safe state. */
    Motor_Init();

#if ROBOT_CFG_APP_MODE_CALIBRATION
    uint32_t i;

    Calibration_Init();
    Calibration_Run();

    gDbgLatestLine = gCalibrationData.latestLineReading;
    gDbgBumpState = gCalibrationData.latestBumpState;
    gDbgUltraLeftBaselineCm = gCalibrationData.ultrasonicLeftBaselineCm;
    gDbgUltraRightBaselineCm = gCalibrationData.ultrasonicRightBaselineCm;

    for (i = 0U; i < 8U; i++) {
        gDbgLineWhite[i] = gCalibrationData.lineWhiteCounts[i];
        gDbgLineBlack[i] = gCalibrationData.lineBlackCounts[i];
        gDbgLineThreshold[i] = gCalibrationData.lineThresholdCounts[i];
    }

    /*
     * Calibration only stage:
     * - line sensor calibration should be performed with the robot over white,
     *   then over black tape/line
     * - ultrasonic calibration should be performed on normal flat floor at the
     *   real mounting angle
     * - these calibration constants will later be used by line following,
     *   obstacle detection, and collision logic
     */
    while (1U) {
        gDbgLatestLine = gCalibrationData.latestLineReading;
        gDbgBumpState = gCalibrationData.latestBumpState;
        gDbgUltraLeftBaselineCm = gCalibrationData.ultrasonicLeftBaselineCm;
        gDbgUltraRightBaselineCm = gCalibrationData.ultrasonicRightBaselineCm;

        for (i = 0U; i < 8U; i++) {
            gDbgLineWhite[i] = gCalibrationData.lineWhiteCounts[i];
            gDbgLineBlack[i] = gCalibrationData.lineBlackCounts[i];
            gDbgLineThreshold[i] = gCalibrationData.lineThresholdCounts[i];
        }

        DelayMs(ROBOT_CFG_MAIN_CAL_WATCH_UPDATE_MS);
    }
#else
#if APP_MODE_VERIFY
    /*
     * Front-sensor verification only:
     * - bump switches are front collision sensors
     * - center IR is front-looking distance sensing
     * - this mode is only checking sensor interpretation, not behavior logic
     */
    Verify_Init();
    Collision_Init();
    Reaction_Init();

    while (1U) {
        Verify_Update();
        gDbgVerifyBumpMask = gVerifyBumpMask;
        gDbgCollisionZone = Collision_FromBumpMask(gVerifyBumpMask);
        Reaction_Execute(gDbgCollisionZone);
        gDbgVerifyBumpIndex = gVerifyBumpIndex;
        gDbgVerifyMultipleBumps = gVerifyMultipleBumps ? 1U : 0U;
        gDbgVerifyFrontIrRaw = gVerifyFrontIrRaw;
        DelayMs(ROBOT_CFG_MAIN_CONTROL_LOOP_MS);
    }
#else
    Calibration_Init();
    Calibration_Run();
#if ROBOT_CFG_APP_MODE == ROBOT_MODE_DEMO
    Control_Init();
    Demo_Init();
#else
    Control_Init();
#endif

    while (1U) {
        gDbgLatestLine = gCalibrationData.latestLineReading;
#if ROBOT_CFG_APP_MODE == ROBOT_MODE_DEMO
        gDbgControlState = gControlState;
        gDbgUltraLeftBaselineCm = gCalibrationData.ultrasonicLeftBaselineCm;
        gDbgUltraRightBaselineCm = gCalibrationData.ultrasonicRightBaselineCm;
        Demo_Update();
        gDbgDemoStage = gDemoStage;
        gDbgDemoBumpMask = gDemoBumpState;
        gDbgDemoCollisionZone = gDemoCollisionZone;
        gDbgDemoLineData = gDemoLineData;
        gDbgDemoFrontIrRaw = gDemoIrRaw;
        gDbgDemoUltraLeftCm = gDemoUltraLeftCm;
        gDbgDemoUltraRightCm = gDemoUltraRightCm;
        gDbgDemoEmergencyActive = gDemoEmergencyActive;
#if ENABLE_UART_DEBUG
        Debug_PrintStateTransitions();
        Debug_PrintLineTuningEvents();
        Debug_PrintLineTuningStatus();
#endif
#else
        gDbgControlState = gControlState;
        Control_Update();
#if ENABLE_UART_DEBUG
        Debug_PrintStateTransitions();
        Debug_PrintLineTuningEvents();
        Debug_PrintLineTuningStatus();
#endif
#endif
        DelayMs(ROBOT_CFG_MAIN_CONTROL_LOOP_MS);
    }
#endif
#endif
}

static void Clock_Init(void)
{
    WDT_A_holdTimer();
    SysTick_Init();
    __enable_irq();
}

void SysTick_Handler(void)
{
    gMsTicks++;
}

static void SysTick_Init(void)
{
    uint32_t mclkHz = CS_getMCLK();
    uint32_t reload;

    if (mclkHz == 0U) {
        mclkHz = 3000000U;
    }

    reload = (mclkHz / 1000U) - 1U;

    SysTick_disableModule();
    SysTick_setPeriod(reload);
    SysTick_enableInterrupt();
    SysTick_enableModule();
}

static void DelayMs(uint32_t ms)
{
    uint32_t target = gMsTicks + ms;

    while ((int32_t)(target - gMsTicks) > 0) {
        __WFI();
    }
}

#if ENABLE_UART_DEBUG
static void DebugUart_Init(void)
{
    GPIO_setAsPeripheralModuleFunctionInputPin(
        DEBUG_UART_RX_PORT,
        DEBUG_UART_RX_PIN,
        GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(
        DEBUG_UART_TX_PORT,
        DEBUG_UART_TX_PIN,
        GPIO_PRIMARY_MODULE_FUNCTION);
    UART_initModule(DEBUG_UART_MODULE, &gDebugUartConfig);
    UART_enableModule(DEBUG_UART_MODULE);
}

static void DebugUart_WriteString(const char *text)
{
    while (*text != '\0') {
        if (*text == '\n') {
            while (UART_getInterruptStatus(
                       DEBUG_UART_MODULE,
                       EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG) == 0U) {
            }
            UART_transmitData(DEBUG_UART_MODULE, '\r');
        }

        while (UART_getInterruptStatus(
                   DEBUG_UART_MODULE,
                   EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG) == 0U) {
        }
        UART_transmitData(DEBUG_UART_MODULE, (uint_fast8_t)(*text));
        text++;
    }
}

static const char *Debug_ControlStateName(ControlState state)
{
    switch (state) {
    case CONTROL_STATE_LINE_FOLLOW:
        return "FOLLOW";
    case CONTROL_STATE_OBSTACLE_AVOID:
        return "OBSTACLE";
    case CONTROL_STATE_TORQUE_RAMP:
        return "TORQUE";
    case CONTROL_STATE_STOP:
        return "STOP";
    case CONTROL_STATE_OBSTACLE_APPROACH_CONFIRM:
        return "OBS_CONFIRM";
    case CONTROL_STATE_EMERGENCY_BUMP:
        return "BUMP";
    case CONTROL_STATE_OBSTACLE_BYPASS:
        return "BYPASS";
    case CONTROL_STATE_LINE_REACQUIRE_GATED:
        return "REACQUIRE";
    case CONTROL_STATE_COLLISION:
        return "COLLISION";
    case CONTROL_STATE_EMERGENCY:
        return "EMERGENCY";
    default:
        return "UNKNOWN";
    }
}

static const char *Debug_AvoidStateName(AvoidState state)
{
    switch (state) {
    case AVOID_STATE_IDLE:
        return "IDLE";
    case AVOID_STATE_OBSTACLE_APPROACH_CONFIRM:
        return "OBS_CONFIRM";
    case AVOID_STATE_EMERGENCY_BUMP:
        return "BUMP";
    case AVOID_STATE_OBSTACLE_BYPASS:
        return "BYPASS";
    case AVOID_STATE_LINE_REACQUIRE_GATED:
        return "REACQUIRE";
    default:
        return "UNKNOWN";
    }
}

static const char *Debug_LineFollowModeName(unsigned int mode)
{
    switch ((LineFollowMode)mode) {
    case LINE_FOLLOW_MODE_FOLLOW_LINE:
        return "FOLLOW";
    case LINE_FOLLOW_MODE_INTERSECTION_LOCK:
        return "LOCK";
    case LINE_FOLLOW_MODE_WIDE_LINE_STABILISE:
        return "WIDE";
    case LINE_FOLLOW_MODE_GAP_BRIDGE:
        return "GAP";
    case LINE_FOLLOW_MODE_RAMP_TRAVERSE:
        return "RAMP";
    case LINE_FOLLOW_MODE_RECOVER_SEARCH:
        return "RECOVER";
    case LINE_FOLLOW_MODE_FAILSAFE_STOP:
        return "STOP";
    default:
        return "UNKNOWN";
    }
}

static const char *Debug_LineMotionName(unsigned int motion)
{
    switch (motion) {
    case 1U:
        return "FWD";
    case 2U:
        return "LEFT";
    case 3U:
        return "RIGHT";
    case 4U:
        return "BACK";
    default:
        return "STOP";
    }
}

static const char *Debug_MissionTorquePhaseName(unsigned int phase)
{
    switch ((MissionTorquePhase)phase) {
    case MISSION_TORQUE_PHASE_ENTRY:
        return "ENTRY";
    case MISSION_TORQUE_PHASE_CLIMB:
        return "CLIMB";
    case MISSION_TORQUE_PHASE_DESCENT:
        return "DESC";
    case MISSION_TORQUE_PHASE_EXIT_SEARCH:
        return "SEARCH";
    case MISSION_TORQUE_PHASE_COMPLETE:
        return "DONE";
    case MISSION_TORQUE_PHASE_FAULT_STOP:
        return "FAULT";
    default:
        return "UNK";
    }
}

static void Debug_PrintStateTransitions(void)
{
#if ENABLE_UART_TUNING_DEBUG && ENABLE_STATE_PRINTS
    char buffer[128];

    if (gDbgPrevPrintControlState != gControlState) {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "STATE %s -> %s\n",
            Debug_ControlStateName((ControlState)gDbgPrevPrintControlState),
            Debug_ControlStateName(gControlState));
        DebugUart_WriteString(buffer);
        gDbgPrevPrintControlState = gControlState;
    }

    if (gDbgPrevPrintAvoidState != gAvoidState) {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "AVOID %s -> %s\n",
            Debug_AvoidStateName((AvoidState)gDbgPrevPrintAvoidState),
            Debug_AvoidStateName(gAvoidState));
        DebugUart_WriteString(buffer);
        gDbgPrevPrintAvoidState = gAvoidState;
    }

    if ((gDbgPrevMissionWatchdog == 0U) && (gMissionWatchdogTripped != 0U)) {
        DebugUart_WriteString("MISSION watchdog stop\n");
    }

    if ((gDbgPrevMissionTimeout == 0U) && (gMissionTimeoutStop != 0U)) {
        DebugUart_WriteString("MISSION timeout stop\n");
    }

    if (gDbgPrevMissionZoneSwitches != gMissionZoneSwitches) {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "MISSION zone=%u mode=%s\n",
            gMissionZoneSwitches,
            Debug_ControlStateName((ControlState)gMissionMode));
        DebugUart_WriteString(buffer);
    }

    if (gDbgPrevMissionTorquePhase != gMissionTorquePhase) {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "TRQ phase %s -> %s p=%d y=%d\n",
            Debug_MissionTorquePhaseName(gDbgPrevMissionTorquePhase),
            Debug_MissionTorquePhaseName(gMissionTorquePhase),
            gMissionTorquePitchDeg,
            gMissionTorqueYawDeg);
        DebugUart_WriteString(buffer);
    }

    if ((gDbgPrevMissionTorqueFault != gMissionTorqueFault) &&
        (gMissionTorqueFault != 0U)) {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "TRQ fault=%u p=%d lc=%u\n",
            gMissionTorqueFault,
            gMissionTorquePitchDeg,
            gMissionTorqueLineCount);
        DebugUart_WriteString(buffer);
    }

    gDbgPrevMissionWatchdog = gMissionWatchdogTripped;
    gDbgPrevMissionTimeout = gMissionTimeoutStop;
    gDbgPrevMissionZoneSwitches = gMissionZoneSwitches;
    gDbgPrevMissionTorquePhase = gMissionTorquePhase;
    gDbgPrevMissionTorqueFault = gMissionTorqueFault;

    if (gDbgPrevAvoidAttempts != gAvoidAttemptCount) {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "OBS try=%u hz=%u side=%u\n",
            gAvoidAttemptCount,
            gAvoidObsConf,
            gAvoidChosenBypassSide);
        DebugUart_WriteString(buffer);
    }

    if (gDbgPrevAvoidBumpEvents != gAvoidBumpEventCount) {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "OBS bump=%u mask=0x%02X\n",
            gAvoidBumpEventCount,
            gAvoidBumpMask6);
        DebugUart_WriteString(buffer);
    }

    if (gDbgPrevAvoidNearMiss != gAvoidNearMissCount) {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "OBS near=%u hz=%u ir=%u us=%u\n",
            gAvoidNearMissCount,
            gAvoidObsConf,
            gAvoidIrMm,
            gAvoidMinUsCm);
        DebugUart_WriteString(buffer);
    }

    if (gDbgPrevAvoidSuccess != gAvoidSuccessCount) {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "OBS ok=%u\n",
            gAvoidSuccessCount);
        DebugUart_WriteString(buffer);
    }

    if (gDbgPrevAvoidTimeouts != gAvoidTimeoutCount) {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "OBS timeout=%u\n",
            gAvoidTimeoutCount);
        DebugUart_WriteString(buffer);
    }

    gDbgPrevAvoidNearMiss = gAvoidNearMissCount;
    gDbgPrevAvoidBumpEvents = gAvoidBumpEventCount;
    gDbgPrevAvoidAttempts = gAvoidAttemptCount;
    gDbgPrevAvoidSuccess = gAvoidSuccessCount;
    gDbgPrevAvoidTimeouts = gAvoidTimeoutCount;
#endif
}

static void Debug_PrintLineTuningEvents(void)
{
#if ENABLE_UART_TUNING_DEBUG
    char buffer[128];

    if ((gDbgPrevStartupLatched == 0U) && (gLineFollowStartupLineLatched != 0U)) {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "START line=1 cnt=%u err=%d\n",
            gLineFollowCount,
            gLineFollowErr);
        DebugUart_WriteString(buffer);
    } else if ((gDbgPrevStartupValid == 0U) &&
               (gLineFollowLineValidAtStartup != 0U)) {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "START line=1 cnt=%u err=%d\n",
            gLineFollowCount,
            gLineFollowErr);
        DebugUart_WriteString(buffer);
    }

    if ((gDbgPrevPidFirstSample != 0U) && (gLineFollowPidFirstSample == 0U)) {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "PIDINIT e=%d de=%d\n",
            gLineFollowPidErr,
            gLineFollowPidDErr);
        DebugUart_WriteString(buffer);
    }

    if (gDbgPrevLineMode != gLineFollowCurrentState) {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "LFMODE %s -> %s rs=%u mk=0x%02X lc=%u le=%d\n",
            Debug_LineFollowModeName(gDbgPrevLineMode),
            Debug_LineFollowModeName(gLineFollowCurrentState),
            gLineFollowPidResetReason,
            gLineFollowMask,
            gLineFollowCount,
            gLineFollowErr);
        DebugUart_WriteString(buffer);
    }

    if ((gDbgPrevPidClamped == 0U) && (gLineFollowPidClamped != 0U)) {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "CLAMP corr=%d\n",
            gLineFollowPidCorr);
        DebugUart_WriteString(buffer);
    }

    gDbgPrevStartupLatched = gLineFollowStartupLineLatched;
    gDbgPrevStartupValid = gLineFollowLineValidAtStartup;
    gDbgPrevLineMode = gLineFollowCurrentState;
    gDbgPrevPidFirstSample = gLineFollowPidFirstSample;
    gDbgPrevLineValid = (gLineFollowCount > 0U) ? 1U : 0U;
    gDbgPrevPidClamped = gLineFollowPidClamped;
#endif
}

static void Debug_PrintLineTuningStatus(void)
{
#if ENABLE_UART_TUNING_DEBUG && ENABLE_SENSOR_PRINTS
    char buffer[192];

    if (UART_DEBUG_EVERY_N_TICKS == 0U) {
        return;
    }

    gDbgPrintLoopCount++;
    if (gDbgPrintLoopCount < UART_DEBUG_EVERY_N_TICKS) {
        return;
    }
    gDbgPrintLoopCount = 0U;

    if (gMissionMode == CONTROL_STATE_TORQUE_RAMP) {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "TRQ ph=%s p=%d y=%d ye=%d yc=%d lc=%u L=%u R=%u b=%u\n",
            Debug_MissionTorquePhaseName(gMissionTorquePhase),
            gMissionTorquePitchDeg,
            gMissionTorqueYawDeg,
            gMissionTorqueYawErr,
            gMissionTorqueYawCorr,
            gMissionTorqueLineCount,
            gMissionTorqueLeftDuty,
            gMissionTorqueRightDuty,
            gMissionTorqueDuty);
    } else {
        (void)snprintf(
            buffer,
            sizeof(buffer),
            "LF md=%s mk=0x%02X lc=%u le=%d lp=%u rc=%u mv=%s L=%u R=%u c=%d\n",
            Debug_LineFollowModeName(gLineFollowCurrentState),
            gLineFollowMask,
            gLineFollowCount,
            gLineFollowErr,
            (gLineFollowCount > 0U) ? 1U : 0U,
            gLineFollowRecoverState,
            Debug_LineMotionName(gLineFollowCmdMotion),
            gLineFollowCmdLeftDuty,
            gLineFollowCmdRightDuty,
            gLineFollowCorr);
    }
    DebugUart_WriteString(buffer);
#endif
}
#endif
