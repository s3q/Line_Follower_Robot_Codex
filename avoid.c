#include "avoid.h"
#include "bump.h"
#include "collision.h"
#include "ir.h"
#include "line.h"
#include "line_follow.h"
#include "line_follow_mode.h"
#include "motor.h"
#include "obstacle_mode.h"
#include "robot_config.h"
#include "ultrasonic.h"

typedef enum {
    AVOID_STEP_NONE = 0,
    AVOID_STEP_SETTLE = 1,
    AVOID_STEP_BACKOFF = 2,
    AVOID_STEP_PIVOT_AWAY = 3,
    AVOID_STEP_FORWARD_CLEAR = 4,
    AVOID_STEP_PIVOT_BACK = 5,
    AVOID_STEP_BOUNDARY_FOLLOW = 6
} AvoidStep;

volatile AvoidState gAvoidState = AVOID_STATE_IDLE;
volatile unsigned int gAvoidObsConf = 0U;
volatile unsigned int gAvoidObsConfirmCount = 0U;
volatile unsigned int gAvoidObstacleWarnCount = 0U;
volatile unsigned int gAvoidObstacleClearCount = 0U;
volatile unsigned int gAvoidIrMm = 0U;
volatile unsigned int gAvoidSymmetryMetricCm = 0U;
volatile unsigned int gAvoidBumpMask6 = 0U;
volatile unsigned int gAvoidBumpZone = COLLISION_NONE;
volatile unsigned int gAvoidChosenBypassSide = AVOID_BYPASS_SIDE_NONE;
volatile unsigned int gAvoidBackoffRemainingMs = 0U;
volatile unsigned int gAvoidRepeatedBumpCount = 0U;
volatile unsigned int gAvoidReacquireEnabled = 0U;
volatile unsigned int gAvoidReacquireStableCount = 0U;
volatile unsigned int gAvoidBypassPhase = AVOID_STEP_NONE;
volatile unsigned int gAvoidBypassTimerMs = 0U;
volatile unsigned int gAvoidMinUsCm = 0U;
volatile unsigned int gAvoidObstacleConfirmed = 0U;
volatile unsigned int gAvoidObstacleSuppressedByRamp = 0U;
volatile float gAvoidUsLeftCm = -1.0f;
volatile float gAvoidUsRightCm = -1.0f;
volatile unsigned int gAvoidAiObstacleValid = 0U;
volatile unsigned int gAvoidAiBypassSide = AVOID_BYPASS_SIDE_NONE;
volatile unsigned int gAvoidAiObstacleClass = 0U;
volatile unsigned int gAvoidAiConfidence = 0U;
volatile unsigned int gAvoidAiDataAgeTicks = 0U;
volatile unsigned int gAvoidAiFresh = 0U;
volatile unsigned int gAvoidStep = AVOID_STEP_NONE;
volatile unsigned int gAvoidStepTimerMs = 0U;
volatile unsigned int gAvoidClearExtendCounter = 0U;
volatile int gAvoidSideErr = 0;
volatile int gAvoidPrevSideErr = 0;
volatile int gAvoidCorrSide = 0;
volatile unsigned int gAvoidBaseDuty = 0U;
volatile unsigned int gAvoidBoundaryModeActive = 0U;
volatile unsigned int gAvoidFrontBlocked = 0U;
volatile unsigned int gAvoidAvoidTimerMs = 0U;
volatile int gAvoidYawInputDeg = 0;
volatile unsigned int gAvoidYawInputFresh = 0U;
volatile int gAvoidEntryYaw = 0;
volatile int gAvoidYawNow = 0;
volatile int gAvoidYawDelta = 0;
volatile unsigned int gAvoidLineAcceptRejectReason = 0U;
volatile unsigned int gAvoidReacquireAcceptCount = 0U;
volatile unsigned int gAvoidReacquireSweepStage = 0U;
volatile unsigned int gAvoidReacquireStepTimerMs = 0U;
volatile unsigned int gAvoidLineCount = 0U;
volatile int gAvoidLineErr = 0;
volatile unsigned int gAvoidFrontMinRangeCm = 0U;
volatile unsigned int gAvoidReacquireGatePassed = 0U;
volatile unsigned int gAvoidHeadingSource = 0U;
volatile unsigned int gAvoidNearMissCount = 0U;
volatile unsigned int gAvoidBumpEventCount = 0U;
volatile unsigned int gAvoidAttemptCount = 0U;
volatile unsigned int gAvoidSuccessCount = 0U;
volatile unsigned int gAvoidTimeoutCount = 0U;

#if ENABLE_OBSTACLE_FEATURE
static ObstacleModeContext gObstacleCtx;
static ObstacleModeInput gObstacleInput;
static ObstacleModeOutput gObstacleOutput;
#endif
static unsigned int gAvoidDecisionReady = 0U;

#if ENABLE_OBSTACLE_FEATURE
static unsigned int Avoid_FloatCmToUint(float valueCm);
static unsigned int Avoid_IrRawToMm(unsigned int irRaw);
static unsigned int Avoid_CountLineBits(uint8_t mask);
static int Avoid_WrapAngleDeg(int angleDeg);
static unsigned int Avoid_StateToBypassPhase(ObstacleModeState state);
static AvoidState Avoid_StateToLegacyState(ObstacleModeState state);
static void Avoid_UpdateInputSnapshot(void);
static void Avoid_MapOutputToDebug(void);
static void Avoid_ApplyMotorCommand(void);
#endif

void Avoid_Init(void)
{
    Ultrasonic_Init();
    IR_Init();
    Line_Init();
    Bump_Init();

    gAvoidState = AVOID_STATE_IDLE;
    gAvoidObsConf = 0U;
    gAvoidObsConfirmCount = 0U;
    gAvoidObstacleWarnCount = 0U;
    gAvoidObstacleClearCount = 0U;
    gAvoidIrMm = 0U;
    gAvoidSymmetryMetricCm = 0U;
    gAvoidBumpMask6 = 0U;
    gAvoidBumpZone = COLLISION_NONE;
    gAvoidChosenBypassSide = AVOID_BYPASS_SIDE_NONE;
    gAvoidBackoffRemainingMs = 0U;
    gAvoidRepeatedBumpCount = 0U;
    gAvoidReacquireEnabled = 0U;
    gAvoidReacquireStableCount = 0U;
    gAvoidBypassPhase = AVOID_STEP_NONE;
    gAvoidBypassTimerMs = 0U;
    gAvoidMinUsCm = 0U;
    gAvoidObstacleConfirmed = 0U;
    gAvoidObstacleSuppressedByRamp = 0U;
    gAvoidUsLeftCm = -1.0f;
    gAvoidUsRightCm = -1.0f;
    gAvoidAiObstacleValid = 0U;
    gAvoidAiBypassSide = AVOID_BYPASS_SIDE_NONE;
    gAvoidAiObstacleClass = 0U;
    gAvoidAiConfidence = 0U;
    gAvoidAiDataAgeTicks = 0U;
    gAvoidAiFresh = 0U;
    gAvoidStep = AVOID_STEP_NONE;
    gAvoidStepTimerMs = 0U;
    gAvoidClearExtendCounter = 0U;
    gAvoidSideErr = 0;
    gAvoidPrevSideErr = 0;
    gAvoidCorrSide = 0;
    gAvoidBaseDuty = 0U;
    gAvoidBoundaryModeActive = 0U;
    gAvoidFrontBlocked = 0U;
    gAvoidAvoidTimerMs = 0U;
    gAvoidEntryYaw = 0;
    gAvoidYawNow = 0;
    gAvoidYawDelta = 0;
    gAvoidLineAcceptRejectReason = 0U;
    gAvoidReacquireAcceptCount = 0U;
    gAvoidReacquireSweepStage = 0U;
    gAvoidReacquireStepTimerMs = 0U;
    gAvoidLineCount = 0U;
    gAvoidLineErr = 0;
    gAvoidFrontMinRangeCm = 0U;
    gAvoidReacquireGatePassed = 0U;
    gAvoidHeadingSource = 0U;
    gAvoidNearMissCount = 0U;
    gAvoidBumpEventCount = 0U;
    gAvoidAttemptCount = 0U;
    gAvoidSuccessCount = 0U;
    gAvoidTimeoutCount = 0U;
    gAvoidDecisionReady = 0U;

#if ENABLE_OBSTACLE_FEATURE
    ObstacleMode_Init(&gObstacleCtx);
#endif
}

bool Avoid_ShouldOverride(void)
{
#if !ENABLE_OBSTACLE_FEATURE
    gAvoidState = AVOID_STATE_IDLE;
    gAvoidDecisionReady = 0U;
    return false;
#else
    Avoid_UpdateInputSnapshot();
    ObstacleMode_Update(&gObstacleCtx, &gObstacleInput, &gObstacleOutput);
    Avoid_MapOutputToDebug();
    gAvoidDecisionReady = 1U;
    return (gObstacleOutput.overrideActive != 0U);
#endif
}

void Avoid_Update(void)
{
#if !ENABLE_OBSTACLE_FEATURE
    return;
#else
    if (gAvoidDecisionReady == 0U) {
        Avoid_UpdateInputSnapshot();
        ObstacleMode_Update(&gObstacleCtx, &gObstacleInput, &gObstacleOutput);
        Avoid_MapOutputToDebug();
    }

    Avoid_ApplyMotorCommand();
    gAvoidDecisionReady = 0U;
#endif
}

#if ENABLE_OBSTACLE_FEATURE
static unsigned int Avoid_FloatCmToUint(float valueCm)
{
    if (valueCm <= 0.0f) {
        return 0U;
    }
    return (unsigned int)(valueCm + 0.5f);
}

static unsigned int Avoid_IrRawToMm(unsigned int irRaw)
{
    if (irRaw == 0U) {
        return 999U;
    }
    return 800000U / irRaw;
}

static unsigned int Avoid_CountLineBits(uint8_t mask)
{
    unsigned int count = 0U;
    while (mask != 0U) {
        count += (unsigned int)(mask & 1U);
        mask >>= 1;
    }
    return count;
}

static int Avoid_WrapAngleDeg(int angleDeg)
{
    while (angleDeg > 180) {
        angleDeg -= 360;
    }
    while (angleDeg < -180) {
        angleDeg += 360;
    }
    return angleDeg;
}

static unsigned int Avoid_StateToBypassPhase(ObstacleModeState state)
{
    switch (state) {
    case OBSTACLE_MODE_CONFIRM:
        return AVOID_STEP_SETTLE;
    case OBSTACLE_MODE_EMERGENCY_BACKOFF:
        return AVOID_STEP_BACKOFF;
    case OBSTACLE_MODE_TURN_OUT:
        return AVOID_STEP_PIVOT_AWAY;
    case OBSTACLE_MODE_BYPASS_FORWARD:
        return AVOID_STEP_FORWARD_CLEAR;
    case OBSTACLE_MODE_SEARCH_LINE:
        return AVOID_STEP_PIVOT_BACK;
    case OBSTACLE_MODE_NO_LINE_CRUISE:
        return AVOID_STEP_FORWARD_CLEAR;
    case OBSTACLE_MODE_EXIT_CONFIRM:
        return AVOID_STEP_SETTLE;
    case OBSTACLE_MODE_TIMEOUT_STOP:
    case OBSTACLE_MODE_IDLE:
    default:
        return AVOID_STEP_NONE;
    }
}

static AvoidState Avoid_StateToLegacyState(ObstacleModeState state)
{
    switch (state) {
    case OBSTACLE_MODE_CONFIRM:
        return AVOID_STATE_OBSTACLE_APPROACH_CONFIRM;
    case OBSTACLE_MODE_EMERGENCY_BACKOFF:
        return AVOID_STATE_EMERGENCY_BUMP;
    case OBSTACLE_MODE_TURN_OUT:
    case OBSTACLE_MODE_BYPASS_FORWARD:
        return AVOID_STATE_OBSTACLE_BYPASS;
    case OBSTACLE_MODE_SEARCH_LINE:
    case OBSTACLE_MODE_EXIT_CONFIRM:
    case OBSTACLE_MODE_NO_LINE_CRUISE:
    case OBSTACLE_MODE_TIMEOUT_STOP:
        return AVOID_STATE_LINE_REACQUIRE_GATED;
    case OBSTACLE_MODE_IDLE:
    default:
        return AVOID_STATE_IDLE;
    }
}

static void Avoid_UpdateInputSnapshot(void)
{
    uint8_t lineMask;
    unsigned int usLeftCm;
    unsigned int usRightCm;
    unsigned int irRaw;
    unsigned int lineCount;
    int lineErr;

    gAvoidUsLeftCm = Ultrasonic_ReadLeftCm();
    gAvoidUsRightCm = Ultrasonic_ReadRightCm();
    usLeftCm = Avoid_FloatCmToUint(gAvoidUsLeftCm);
    usRightCm = Avoid_FloatCmToUint(gAvoidUsRightCm);

    if ((usLeftCm == 0U) || ((usRightCm != 0U) && (usRightCm < usLeftCm))) {
        gAvoidMinUsCm = usRightCm;
    } else {
        gAvoidMinUsCm = usLeftCm;
    }

    if (usLeftCm >= usRightCm) {
        gAvoidSymmetryMetricCm = usLeftCm - usRightCm;
    } else {
        gAvoidSymmetryMetricCm = usRightCm - usLeftCm;
    }
    gAvoidFrontMinRangeCm = gAvoidMinUsCm;

    irRaw = IR_ReadRaw();
    gAvoidIrMm = Avoid_IrRawToMm(irRaw);

    gAvoidBumpMask6 = Bump_Read();
    gAvoidBumpZone = Collision_FromBumpMask((uint8_t)gAvoidBumpMask6);

    lineMask = Line_Read(ROBOT_CFG_LINE_FOLLOW_SAMPLE_US);
    lineCount = Avoid_CountLineBits(lineMask);
    lineErr = (int)LineFollowMode_ComputeLineError((uint8_t)lineMask, (uint8_t)lineCount);
    gAvoidLineCount = lineCount;
    gAvoidLineErr = lineErr;

    if (gAvoidYawInputFresh != 0U) {
        gAvoidYawNow = Avoid_WrapAngleDeg(gAvoidYawInputDeg);
        gAvoidHeadingSource = 1U;
    } else {
        gAvoidHeadingSource = 2U;
    }

    gObstacleInput.irMm = (uint16_t)gAvoidIrMm;
    gObstacleInput.usLeftCm = (uint16_t)usLeftCm;
    gObstacleInput.usRightCm = (uint16_t)usRightCm;
    gObstacleInput.bumpMask = (uint8_t)gAvoidBumpMask6;
    gObstacleInput.lineCount = (uint8_t)lineCount;
    gObstacleInput.lineErr = (int16_t)lineErr;
    gObstacleInput.yawDeg = (int16_t)gAvoidYawNow;
    gObstacleInput.yawValid = (uint8_t)((gAvoidYawInputFresh != 0U) ? 1U : 0U);
#if ENABLE_RAMP_FEATURE
    gObstacleInput.rampLikely = (uint8_t)((gLineFollowRampConf >= ROBOT_CFG_RAMP_N) ? 1U : 0U);
#else
    gObstacleInput.rampLikely = 0U;
#endif
}

static void Avoid_MapOutputToDebug(void)
{
    unsigned int timeoutCycles;
    unsigned int remainingCycles;

    gAvoidState = Avoid_StateToLegacyState(gObstacleOutput.state);
    gAvoidBypassPhase = Avoid_StateToBypassPhase(gObstacleOutput.state);
    gAvoidStep = gAvoidBypassPhase;
    gAvoidStepTimerMs = 0U;
    gAvoidBoundaryModeActive = (gObstacleOutput.motion == OBSTACLE_MOTION_FORWARD) ? 1U : 0U;
    gAvoidFrontBlocked = (gObstacleOutput.frontHazard >= 850U) ? 1U : 0U;
    gAvoidObsConf = gObstacleOutput.frontHazard / 50U;
    if (gAvoidObsConf > ROBOT_CFG_OBS_CONF_MAX) {
        gAvoidObsConf = ROBOT_CFG_OBS_CONF_MAX;
    }
    gAvoidObsConfirmCount = gObstacleCtx.confirmCycles;
    gAvoidObstacleWarnCount = (gObstacleOutput.frontHazard >= 500U) ? gObstacleCtx.stateCycles : 0U;
    gAvoidObstacleClearCount = gObstacleCtx.clearCycles;
    gAvoidObstacleConfirmed = (gObstacleOutput.state >= OBSTACLE_MODE_TURN_OUT) ? 1U : 0U;
    gAvoidObstacleSuppressedByRamp =
        (gObstacleInput.rampLikely != 0U) &&
        (gObstacleOutput.leftHazard == 0U) &&
        (gObstacleOutput.rightHazard == 0U) ? 1U : 0U;
    gAvoidChosenBypassSide = (gObstacleCtx.turnDir == 1U) ? AVOID_BYPASS_SIDE_LEFT : AVOID_BYPASS_SIDE_RIGHT;
    gAvoidBaseDuty = gObstacleOutput.leftDuty;
    gAvoidSideErr = (int)gObstacleOutput.leftHazard - (int)gObstacleOutput.rightHazard;
    gAvoidCorrSide = (int)gObstacleOutput.leftDuty - (int)gObstacleOutput.rightDuty;
    gAvoidPrevSideErr = gAvoidSideErr;
    gAvoidReacquireEnabled =
        ((gObstacleOutput.state == OBSTACLE_MODE_SEARCH_LINE) ||
         (gObstacleOutput.state == OBSTACLE_MODE_EXIT_CONFIRM)) ? 1U : 0U;
    gAvoidReacquireStableCount = gObstacleCtx.reacquireCycles;
    gAvoidReacquireAcceptCount = gObstacleCtx.reacquireCycles;
    gAvoidReacquireSweepStage = gObstacleCtx.searchPhase;
    gAvoidReacquireStepTimerMs = 0U;
    gAvoidReacquireGatePassed = (gObstacleOutput.lineRecovered != 0U) ? 1U : 0U;
    gAvoidLineAcceptRejectReason = (gObstacleOutput.lineRecovered != 0U) ? 0U :
                                   ((gObstacleOutput.state == OBSTACLE_MODE_NO_LINE_CRUISE) ? 8U : 1U);
    gAvoidEntryYaw = gObstacleCtx.yawTargetDeg;
    gAvoidYawDelta = Avoid_WrapAngleDeg(gAvoidYawNow - gAvoidEntryYaw);
    gAvoidBackoffRemainingMs = 0U;
    gAvoidBypassTimerMs = 0U;
    gAvoidClearExtendCounter = gObstacleCtx.stateCycles * ROBOT_CFG_MAIN_CONTROL_LOOP_MS;

    timeoutCycles = ROBOT_CFG_AVOID_TIMEOUT_MS / ROBOT_CFG_MAIN_CONTROL_LOOP_MS;
    if (gObstacleCtx.totalCycles < timeoutCycles) {
        remainingCycles = timeoutCycles - gObstacleCtx.totalCycles;
    } else {
        remainingCycles = 0U;
    }
    gAvoidAvoidTimerMs = remainingCycles * ROBOT_CFG_MAIN_CONTROL_LOOP_MS;

    gAvoidNearMissCount = gObstacleCtx.nearMissCount;
    gAvoidBumpEventCount = gObstacleCtx.bumpCount;
    gAvoidAttemptCount = gObstacleCtx.attemptCount;
    gAvoidSuccessCount = gObstacleCtx.successCount;
    gAvoidTimeoutCount = gObstacleCtx.timeoutCount;
    gAvoidRepeatedBumpCount = gObstacleCtx.bumpCount;
}

static void Avoid_ApplyMotorCommand(void)
{
    switch (gObstacleOutput.motion) {
    case OBSTACLE_MOTION_FORWARD:
        Motor_Forward(gObstacleOutput.leftDuty, gObstacleOutput.rightDuty);
        break;
    case OBSTACLE_MOTION_LEFT:
        Motor_Left(gObstacleOutput.leftDuty, gObstacleOutput.rightDuty);
        break;
    case OBSTACLE_MOTION_RIGHT:
        Motor_Right(gObstacleOutput.leftDuty, gObstacleOutput.rightDuty);
        break;
    case OBSTACLE_MOTION_BACKWARD:
        Motor_Backward(gObstacleOutput.leftDuty, gObstacleOutput.rightDuty);
        break;
    case OBSTACLE_MOTION_STOP:
    default:
        Motor_Stop();
        break;
    }
}
#endif
