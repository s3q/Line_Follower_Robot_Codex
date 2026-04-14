#include "line_follow.h"
#include "line_follow_mode.h"
#include "avoid.h"
#include "ir.h"
#include "line.h"
#include "motor.h"
#include "robot_config.h"
#include "ultrasonic.h"
#include <stdbool.h>
#include <stdint.h>

volatile LineFollowMode gLineFollowMode = LINE_FOLLOW_MODE_FOLLOW_LINE;
volatile unsigned int gLineFollowMask = 0U;
volatile unsigned int gLineFollowCount = 0U;
volatile int gLineFollowErr = 0;
volatile int gLineFollowErrPrev = 0;
volatile unsigned int gLineFollowBaseDuty = 0U;
volatile int gLineFollowCorr = 0;
volatile float gLineFollowUsL = 0.0f;
volatile float gLineFollowUsR = 0.0f;
volatile unsigned int gLineFollowIrRaw = 0U;
volatile unsigned int gLineFollowIrMm = 0U;
volatile int gLineFollowPitchDeg = 0;
volatile int gLineFollowPitchInputDeg = 0;
volatile unsigned int gLineFollowRampConf = 0U;
volatile unsigned int gLineFollowObsConf = 0U;
volatile unsigned int gLineFollowSymmetryMetric = 0U;
volatile int gLineFollowLastLineSide = 0;
volatile unsigned int gLineFollowSweepPhase = 0U;
volatile unsigned int gLineFollowSatTimer = 0U;
volatile unsigned int gLineFollowGapTimer = 0U;
volatile unsigned int gLineFollowGapConf = 0U;
volatile unsigned int gLineFollowIntersectConf = 0U;
volatile unsigned int gLineFollowLockRemainingMs = 0U;
volatile unsigned int gLineFollowMaskContiguousScore = 0U;
volatile unsigned int gLineFollowAvgReflect = 0U;
volatile unsigned int gLineFollowIsRampUp = 0U;
volatile unsigned int gLineFollowIsRampDown = 0U;
volatile unsigned int gLineFollowRampEnterCounter = 0U;
volatile unsigned int gLineFollowRampExitCounter = 0U;
volatile int gLineFollowPidErr = 0;
volatile int gLineFollowPidPrevErr = 0;
volatile int gLineFollowPidDErr = 0;
volatile int gLineFollowPidSumErr = 0;
volatile int gLineFollowPidCorr = 0;
volatile unsigned int gLineFollowPidBaseDuty = 0U;
volatile unsigned int gLineFollowPidLeftDuty = 0U;
volatile unsigned int gLineFollowPidRightDuty = 0U;
volatile int gLineFollowPidPterm = 0;
volatile int gLineFollowPidIterm = 0;
volatile int gLineFollowPidDterm = 0;
volatile unsigned int gLineFollowPidClamped = 0U;
volatile unsigned int gLineFollowPidLineValid = 0U;
volatile unsigned int gLineFollowPidEnabled = 0U;
volatile unsigned int gLineFollowPidIntegralFrozen = 0U;
volatile unsigned int gLineFollowPidResetReason = 0U;
volatile unsigned int gLineFollowLineValidAtStartup = 0U;
volatile unsigned int gLineFollowStartupAcquireCount = 0U;
volatile unsigned int gLineFollowStartupLineLatched = 0U;
volatile unsigned int gLineFollowPidFirstSample = 1U;
volatile unsigned int gLineFollowCurrentState = LINE_FOLLOW_MODE_FOLLOW_LINE;
volatile unsigned int gLineFollowActive = 0U;
volatile unsigned int gLineFollowCmdMotion = 0U;
volatile unsigned int gLineFollowCmdLeftDuty = 0U;
volatile unsigned int gLineFollowCmdRightDuty = 0U;
volatile unsigned int gLineFollowRecoverState = LINE_RECOVERY_IDLE;
volatile unsigned int gLineFollowRecoverCycles = 0U;

static uint8_t gLastLineMask = 0U;
static uint32_t gCenterTicks = 0U;
static uint32_t gGentleTicks = 0U;
static uint32_t gSharpTicks = 0U;
static uint32_t gGapEligibleTicks = 0U;
static uint32_t gGapTimerCycles = 0U;
static uint32_t gIntersectionLockCycles = 0U;
static uint32_t gWideCycles = 0U;
#if ROBOT_CFG_ENABLE_RAMP_FEATURE
static uint32_t gRampUpTicks = 0U;
static uint32_t gRampDownTicks = 0U;
static uint32_t gRampExitTicks = 0U;
#endif
#if ROBOT_CFG_ENABLE_RAMP_FEATURE
static int gRampDirection = 0;
#endif
static int gLastAbsErr = 0;
static int32_t gLineFollowPidSumState = 0;
static unsigned int gLineFollowStartupSettleRemainingMs = 0U;
static unsigned int gLineFollowLostConfirmCount = 0U;
static unsigned int gLineFollowCornerAssistCycles = 0U;
static LineFollowRecoveryContext gLineRecoverCtx;

static uint8_t LineFollow_CountBits(uint8_t mask);
static uint8_t LineFollow_ReadMaskResolved(void);
static uint8_t LineFollow_LongestRun(uint8_t mask);
static int32_t LineFollow_ComputeError(uint8_t mask, uint8_t count);
static bool LineFollow_IsMaskContiguous(uint8_t mask, uint8_t count);
static bool LineFollow_IsFullWidthPattern(uint8_t count, uint8_t longestRun);
static bool LineFollow_IsWidePattern(uint8_t mask, uint8_t count, uint8_t longestRun);
static bool LineFollow_IsExtremeLeft(uint8_t mask);
static bool LineFollow_IsExtremeRight(uint8_t mask);
static uint16_t LineFollow_ClampDuty(int32_t duty);
#if ROBOT_CFG_ENABLE_RAMP_FEATURE
static uint16_t LineFollow_ScaleDuty(uint16_t duty, uint16_t scalePct);
static bool LineFollow_HasNormalLineStats(uint8_t count, uint8_t longestRun, int32_t absErr);
static void LineFollow_UpdateRampTraverse(uint8_t count, uint8_t longestRun, int32_t err);
#endif
static int32_t LineFollow_ClampSigned(int32_t value, int32_t clampAbs);
static unsigned int LineFollow_ComputeSymmetryMetricCm(float usLeft, float usRight);
static void LineFollow_PrimeStartupAcquire(void);
static void LineFollow_ResetPid(uint8_t reason);
static void LineFollow_ApplyStartupHold(void);
static void LineFollow_ApplyPd(int32_t err, uint16_t baseDuty, int32_t corrClamp, bool soften);
static void LineFollow_CommandStop(void);
static void LineFollow_CommandForward(uint16_t leftDuty, uint16_t rightDuty);
static void LineFollow_CommandLeft(uint16_t leftDuty, uint16_t rightDuty);
static void LineFollow_CommandRight(uint16_t leftDuty, uint16_t rightDuty);
static void LineFollow_CommandBackward(uint16_t leftDuty, uint16_t rightDuty);
static void LineFollow_ApplyRecoveryCommand(const LineFollowRecoveryStep *step);
static void LineFollow_StartRecoverSearch(int side);
static void LineFollow_UpdateRecoverSearch(uint8_t count, int32_t err);
static void LineFollow_UpdateFollowState(uint8_t mask, uint8_t count, int32_t err, uint8_t longestRun);

void LineFollow_Init(void)
{
    Line_Init();
    IR_Init();
    Ultrasonic_Init();
    gLastLineMask = 0U;
    gLineFollowMode = LINE_FOLLOW_MODE_FOLLOW_LINE;
    gLineFollowLastLineSide = 0;
    gLineFollowErr = 0;
    gLineFollowErrPrev = 0;
    gCenterTicks = 0U;
    gGentleTicks = 0U;
    gSharpTicks = 0U;
    gGapEligibleTicks = 0U;
    gGapTimerCycles = 0U;
    gIntersectionLockCycles = 0U;
    gWideCycles = 0U;
#if ROBOT_CFG_ENABLE_RAMP_FEATURE
    gRampUpTicks = 0U;
    gRampDownTicks = 0U;
    gRampExitTicks = 0U;
#endif
#if ROBOT_CFG_ENABLE_RAMP_FEATURE
    gRampDirection = 0;
#endif
    gLastAbsErr = 0;
    gLineFollowSatTimer = 0U;
    gLineFollowGapConf = 0U;
    gLineFollowIntersectConf = 0U;
    gLineFollowIrMm = 0U;
    gLineFollowPitchInputDeg = 0;
    gLineFollowSymmetryMetric = 0U;
    gLineFollowIsRampUp = 0U;
    gLineFollowIsRampDown = 0U;
    gLineFollowRampEnterCounter = 0U;
    gLineFollowRampExitCounter = 0U;
    gLineFollowLineValidAtStartup = 0U;
    gLineFollowStartupAcquireCount = 0U;
    gLineFollowStartupLineLatched = 0U;
    gLineFollowPidFirstSample = 1U;
    gLineFollowCurrentState = LINE_FOLLOW_MODE_FOLLOW_LINE;
    gLineFollowActive = 0U;
    gLineFollowCmdMotion = 0U;
    gLineFollowCmdLeftDuty = 0U;
    gLineFollowCmdRightDuty = 0U;
    gLineFollowRecoverState = LINE_RECOVERY_IDLE;
    gLineFollowRecoverCycles = 0U;
    gLineFollowStartupSettleRemainingMs = 0U;
    gLineFollowLostConfirmCount = 0U;
    gLineFollowCornerAssistCycles = 0U;
    LineFollowMode_RecoveryInit(&gLineRecoverCtx);
    LineFollow_ResetPid(1U);
    LineFollow_PrimeStartupAcquire();
}

void LineFollow_Update(void)
{
    uint8_t lineMask;
    uint8_t lineCount;
    uint8_t longestRun;
    int32_t lineErr;

    lineMask = LineFollow_ReadMaskResolved();
    lineCount = LineFollow_CountBits(lineMask);
    longestRun = LineFollow_LongestRun(lineMask);
    lineErr = LineFollow_ComputeError(lineMask, lineCount);

    gLastLineMask = lineMask;
    gLineFollowMask = lineMask;
    gLineFollowCount = lineCount;
    gLineFollowErrPrev = gLineFollowErr;
    gLineFollowErr = (int)lineErr;
    gLineFollowMaskContiguousScore =
        (lineCount > 0U) ? ((unsigned int)longestRun * 100U) / (unsigned int)lineCount : 0U;
    gLineFollowAvgReflect = (unsigned int)lineCount * 1000U;
    gLineFollowUsL = Ultrasonic_ReadLeftCm();
    gLineFollowUsR = Ultrasonic_ReadRightCm();
    gLineFollowIrRaw = IR_ReadRaw();
    gLineFollowIrMm = gLineFollowIrRaw;
#if ROBOT_CFG_ENABLE_RAMP_FEATURE
    gLineFollowPitchDeg = gLineFollowPitchInputDeg;
#else
    gLineFollowPitchDeg = 0;
#endif
    gLineFollowObsConf = gAvoidObsConf;
    gLineFollowSymmetryMetric = LineFollow_ComputeSymmetryMetricCm(gLineFollowUsL, gLineFollowUsR);
    gLineFollowIsRampUp = 0U;
    gLineFollowIsRampDown = 0U;
    gLineFollowCurrentState = (unsigned int)gLineFollowMode;
    gLineFollowActive = (lineCount > 0U) ? 1U : 0U;

#if ROBOT_CFG_ENABLE_RAMP_FEATURE
    if ((gLineFollowPitchDeg > ROBOT_CFG_RAMP_PITCH_ENTER_DEG) && (lineCount > 0U)) {
        gRampUpTicks++;
    } else {
        gRampUpTicks = 0U;
    }

    if ((gLineFollowPitchDeg < -ROBOT_CFG_RAMP_PITCH_ENTER_DEG) && (lineCount > 0U)) {
        gRampDownTicks++;
    } else {
        gRampDownTicks = 0U;
    }

    if (gLineFollowMode == LINE_FOLLOW_MODE_RAMP_TRAVERSE) {
        gLineFollowRampConf = ROBOT_CFG_RAMP_N;
    } else if ((gRampUpTicks >= ROBOT_CFG_RAMP_N) || (gRampDownTicks >= ROBOT_CFG_RAMP_N)) {
        gLineFollowRampConf = ROBOT_CFG_RAMP_N;
    } else {
        gLineFollowRampConf = 0U;
    }

    gLineFollowRampEnterCounter = (gRampUpTicks > gRampDownTicks) ? gRampUpTicks : gRampDownTicks;
    gLineFollowRampExitCounter = gRampExitTicks;
#else
    gLineFollowRampConf = 0U;
    gLineFollowRampEnterCounter = 0U;
    gLineFollowRampExitCounter = 0U;
#endif

    if (lineCount > 0U) {
        if (lineErr < -(ROBOT_CFG_LINE_E_CENTER / 2)) {
            gLineFollowLastLineSide = -1;
        } else if (lineErr > (ROBOT_CFG_LINE_E_CENTER / 2)) {
            gLineFollowLastLineSide = 1;
        }
    }

    switch (gLineFollowMode) {
    case LINE_FOLLOW_MODE_INTERSECTION_LOCK:
        LineFollow_ResetPid(2U);
        LineFollow_CommandForward(ROBOT_CFG_LINE_DUTY_SLOW, ROBOT_CFG_LINE_DUTY_SLOW);
        if (gIntersectionLockCycles > 0U) {
            gIntersectionLockCycles--;
        }
        gLineFollowLockRemainingMs = gIntersectionLockCycles * ROBOT_CFG_MAIN_CONTROL_LOOP_MS;
        if (gIntersectionLockCycles == 0U) {
            gLineFollowMode = LINE_FOLLOW_MODE_FOLLOW_LINE;
        }
        break;

    case LINE_FOLLOW_MODE_WIDE_LINE_STABILISE:
        gWideCycles++;
        LineFollow_ApplyPd(lineErr, ROBOT_CFG_LINE_DUTY_SLOW,
                           ROBOT_CFG_LINE_CORR_CLAMP_NORMAL / 2, true);
        if ((!LineFollow_IsWidePattern(lineMask, lineCount, longestRun)) ||
            (gWideCycles >= ROBOT_CFG_LINE_WIDE_CYCLES)) {
            gWideCycles = 0U;
            gLineFollowMode = LINE_FOLLOW_MODE_FOLLOW_LINE;
        }
        break;

#if ROBOT_CFG_ENABLE_RAMP_FEATURE
    case LINE_FOLLOW_MODE_RAMP_TRAVERSE:
        LineFollow_UpdateRampTraverse(lineCount, longestRun, lineErr);
        break;
#else
    case LINE_FOLLOW_MODE_RAMP_TRAVERSE:
        gLineFollowMode = LINE_FOLLOW_MODE_FOLLOW_LINE;
        LineFollow_UpdateFollowState(lineMask, lineCount, lineErr, longestRun);
        break;
#endif

    case LINE_FOLLOW_MODE_GAP_BRIDGE:
        gGapTimerCycles++;
        gLineFollowGapTimer = gGapTimerCycles * ROBOT_CFG_MAIN_CONTROL_LOOP_MS;
        LineFollow_CommandForward(ROBOT_CFG_LINE_DUTY_CRUISE, ROBOT_CFG_LINE_DUTY_CRUISE);
        if (lineCount > 0U) {
            gGapTimerCycles = 0U;
            gLineFollowGapConf = 0U;
            gLineFollowGapTimer = 0U;
            gLineFollowMode = LINE_FOLLOW_MODE_FOLLOW_LINE;
        } else if (gGapTimerCycles >= ROBOT_CFG_LINE_GAP_MAX_CYCLES) {
            LineFollow_StartRecoverSearch(gLineFollowLastLineSide);
        }
        break;

    case LINE_FOLLOW_MODE_RECOVER_SEARCH:
        LineFollow_UpdateRecoverSearch(lineCount, lineErr);
        break;

    case LINE_FOLLOW_MODE_FAILSAFE_STOP:
        LineFollow_CommandStop();
        if ((lineCount > 0U) && (lineErr > -ROBOT_CFG_LINE_E_RECOVER_EXIT) &&
            (lineErr < ROBOT_CFG_LINE_E_RECOVER_EXIT)) {
            gLineFollowMode = LINE_FOLLOW_MODE_FOLLOW_LINE;
        }
        break;

    case LINE_FOLLOW_MODE_FOLLOW_LINE:
    default:
        LineFollow_UpdateFollowState(lineMask, lineCount, lineErr, longestRun);
        break;
    }

    gLastAbsErr = (lineErr < 0) ? -(int)lineErr : (int)lineErr;
}

static uint8_t LineFollow_CountBits(uint8_t mask)
{
    uint8_t count = 0U;

    while (mask != 0U) {
        count += (uint8_t)(mask & 1U);
        mask >>= 1;
    }

    return count;
}

static uint8_t LineFollow_ReadMaskResolved(void)
{
    uint8_t lineMask;
    uint8_t lineCount;
    uint8_t resolvedMask;
    uint8_t resolvedCount;

    lineMask = Line_Read(ROBOT_CFG_LINE_FOLLOW_SAMPLE_US);
    lineCount = LineFollow_CountBits(lineMask);

    if (lineCount < ROBOT_CFG_LINE_FULL_WIDTH_N) {
        return lineMask;
    }

    resolvedMask = Line_Read(ROBOT_CFG_LINE_FOLLOW_RESOLVE_SAMPLE_US);
    resolvedCount = LineFollow_CountBits(resolvedMask);

    if ((resolvedCount > 0U) && (resolvedCount < lineCount)) {
        return resolvedMask;
    }

    return lineMask;
}

static uint8_t LineFollow_LongestRun(uint8_t mask)
{
    uint8_t longest = 0U;
    uint8_t current = 0U;
    uint8_t bit;

    for (bit = 0U; bit < 8U; bit++) {
        if ((mask & (uint8_t)(1U << bit)) != 0U) {
            current++;
            if (current > longest) {
                longest = current;
            }
        } else {
            current = 0U;
        }
    }

    return longest;
}

static int32_t LineFollow_ComputeError(uint8_t mask, uint8_t count)
{
    return LineFollowMode_ComputeLineError(mask, count);
}

static bool LineFollow_IsMaskContiguous(uint8_t mask, uint8_t count)
{
    return (count > 0U) && (LineFollow_LongestRun(mask) == count);
}

static bool LineFollow_IsFullWidthPattern(uint8_t count, uint8_t longestRun)
{
    return (count >= ROBOT_CFG_LINE_FULL_WIDTH_N) &&
           (longestRun == count);
}

static bool LineFollow_IsWidePattern(uint8_t mask, uint8_t count, uint8_t longestRun)
{
    return (count >= 4U) && (count <= 6U) &&
           LineFollow_IsMaskContiguous(mask, count) &&
           (longestRun == count);
}

static bool LineFollow_IsExtremeLeft(uint8_t mask)
{
    return ((mask & 0x03U) != 0U) && ((mask & (uint8_t)~0x03U) == 0U);
}

static bool LineFollow_IsExtremeRight(uint8_t mask)
{
    return ((mask & 0xC0U) != 0U) && ((mask & 0x3FU) == 0U);
}

static uint16_t LineFollow_ClampDuty(int32_t duty)
{
    if (duty < (int32_t)ROBOT_CFG_LINE_DUTY_MIN) {
        return ROBOT_CFG_LINE_DUTY_MIN;
    }
    if (duty > (int32_t)ROBOT_CFG_LINE_DUTY_FAST) {
        return ROBOT_CFG_LINE_DUTY_FAST;
    }
    return (uint16_t)duty;
}

static int32_t LineFollow_ClampSigned(int32_t value, int32_t clampAbs)
{
    if (value > clampAbs) {
        return clampAbs;
    }
    if (value < -clampAbs) {
        return -clampAbs;
    }

    return value;
}

static unsigned int LineFollow_ComputeSymmetryMetricCm(float usLeft, float usRight)
{
    float diff;

    diff = usLeft - usRight;
    if (diff < 0.0f) {
        diff = -diff;
    }

    return (unsigned int)(diff + 0.5f);
}

static void LineFollow_PrimeStartupAcquire(void)
{
#if ENABLE_STARTUP_LINE_ACQUIRE_FIX
    uint8_t lineMask;
    uint8_t lineCount;
    uint8_t sample;
    int32_t lineErr;

    gLineFollowStartupAcquireCount = 0U;

    for (sample = 0U; sample < STARTUP_ACQUIRE_N; sample++) {
        lineMask = LineFollow_ReadMaskResolved();
        lineCount = LineFollow_CountBits(lineMask);
        if (lineCount > 0U) {
            gLineFollowStartupAcquireCount++;
            lineErr = LineFollow_ComputeError(lineMask, lineCount);
            gLastLineMask = lineMask;
            gLineFollowMask = lineMask;
            gLineFollowCount = lineCount;
            gLineFollowErr = (int)lineErr;
            gLineFollowErrPrev = (int)lineErr;
            if (lineErr < 0) {
                gLineFollowLastLineSide = -1;
            } else if (lineErr > 0) {
                gLineFollowLastLineSide = 1;
            } else {
                gLineFollowLastLineSide = 0;
            }
        } else {
            gLineFollowStartupAcquireCount = 0U;
        }
    }

    if (gLineFollowStartupAcquireCount >= STARTUP_ACQUIRE_N) {
        gLineFollowLineValidAtStartup = 1U;
        gLineFollowStartupLineLatched = 1U;
        gLineFollowActive = 1U;
        gLineFollowPidFirstSample = 0U;
        gLineFollowStartupSettleRemainingMs = STARTUP_SETTLE_MS;
    }
#endif
}

static void LineFollow_ResetPid(uint8_t reason)
{
    gLineFollowPidSumState = 0;
    gLineFollowPidFirstSample = 1U;
#if ENABLE_PID_DEBUG_WATCH
    gLineFollowPidErr = 0;
    gLineFollowPidPrevErr = 0;
    gLineFollowPidDErr = 0;
    gLineFollowPidSumErr = 0;
    gLineFollowPidCorr = 0;
    gLineFollowPidBaseDuty = 0U;
    gLineFollowPidLeftDuty = 0U;
    gLineFollowPidRightDuty = 0U;
    gLineFollowPidPterm = 0;
    gLineFollowPidIterm = 0;
    gLineFollowPidDterm = 0;
    gLineFollowPidClamped = 0U;
    gLineFollowPidLineValid = 0U;
    gLineFollowPidIntegralFrozen = 0U;
    gLineFollowPidResetReason = reason;
#endif
#if ENABLE_PID_LINE_FOLLOW
    gLineFollowPidEnabled = 1U;
#else
    gLineFollowPidEnabled = 0U;
#endif
}

static void LineFollow_CommandStop(void)
{
    gLineFollowCmdMotion = 0U;
    gLineFollowCmdLeftDuty = 0U;
    gLineFollowCmdRightDuty = 0U;
    Motor_Stop();
}

static void LineFollow_CommandForward(uint16_t leftDuty, uint16_t rightDuty)
{
    gLineFollowCmdMotion = 1U;
    gLineFollowCmdLeftDuty = leftDuty;
    gLineFollowCmdRightDuty = rightDuty;
    Motor_Forward(leftDuty, rightDuty);
}

static void LineFollow_CommandLeft(uint16_t leftDuty, uint16_t rightDuty)
{
    gLineFollowCmdMotion = 2U;
    gLineFollowCmdLeftDuty = leftDuty;
    gLineFollowCmdRightDuty = rightDuty;
    Motor_Left(leftDuty, rightDuty);
}

static void LineFollow_CommandRight(uint16_t leftDuty, uint16_t rightDuty)
{
    gLineFollowCmdMotion = 3U;
    gLineFollowCmdLeftDuty = leftDuty;
    gLineFollowCmdRightDuty = rightDuty;
    Motor_Right(leftDuty, rightDuty);
}

static void LineFollow_CommandBackward(uint16_t leftDuty, uint16_t rightDuty)
{
    gLineFollowCmdMotion = 4U;
    gLineFollowCmdLeftDuty = leftDuty;
    gLineFollowCmdRightDuty = rightDuty;
    Motor_Backward(leftDuty, rightDuty);
}

static void LineFollow_ApplyRecoveryCommand(const LineFollowRecoveryStep *step)
{
    switch (step->command) {
    case LINE_RECOVERY_CMD_FORWARD:
        LineFollow_CommandForward(step->leftDuty, step->rightDuty);
        break;
    case LINE_RECOVERY_CMD_LEFT:
        LineFollow_CommandLeft(step->leftDuty, step->rightDuty);
        break;
    case LINE_RECOVERY_CMD_RIGHT:
        LineFollow_CommandRight(step->leftDuty, step->rightDuty);
        break;
    case LINE_RECOVERY_CMD_BACKWARD:
        LineFollow_CommandBackward(step->leftDuty, step->rightDuty);
        break;
    case LINE_RECOVERY_CMD_STOP:
    default:
        LineFollow_CommandStop();
        break;
    }
}

static void LineFollow_ApplyStartupHold(void)
{
    uint16_t startDuty;

    startDuty = PID_BASE_DUTY;
    if (startDuty < ROBOT_CFG_LINE_START_BOOST_DUTY) {
        startDuty = ROBOT_CFG_LINE_START_BOOST_DUTY;
    }

    gLineFollowBaseDuty = startDuty;
    gLineFollowCorr = 0;
#if ENABLE_PID_DEBUG_WATCH
    gLineFollowPidErr = 0;
    gLineFollowPidPrevErr = 0;
    gLineFollowPidDErr = 0;
    gLineFollowPidSumErr = 0;
    gLineFollowPidCorr = 0;
    gLineFollowPidPterm = 0;
    gLineFollowPidIterm = 0;
    gLineFollowPidDterm = 0;
    gLineFollowPidClamped = 0U;
    gLineFollowPidLineValid = 1U;
    gLineFollowPidBaseDuty = startDuty;
    gLineFollowPidLeftDuty = startDuty;
    gLineFollowPidRightDuty = startDuty;
#endif
    LineFollow_CommandForward(startDuty, startDuty);
}

#if ROBOT_CFG_ENABLE_RAMP_FEATURE
static uint16_t LineFollow_ScaleDuty(uint16_t duty, uint16_t scalePct)
{
    int32_t scaledDuty;

    scaledDuty = ((int32_t)duty * (int32_t)scalePct) / 100;
    return LineFollow_ClampDuty(scaledDuty);
}

static bool LineFollow_HasNormalLineStats(uint8_t count, uint8_t longestRun, int32_t absErr)
{
    return (count >= 1U) && (count <= 3U) &&
           (longestRun == count) &&
           (absErr < ROBOT_CFG_LINE_E_SHARP);
}
#endif

static void LineFollow_ApplyPd(int32_t err, uint16_t baseDuty, int32_t corrClamp, bool soften)
{
    int32_t derr;
    int32_t corr;
    int32_t leftDuty;
    int32_t rightDuty;
    float nearestUs;
#if ENABLE_PID_LINE_FOLLOW
    int32_t effectiveErr;
    int32_t sumCandidate;
    int32_t pRaw;
    int32_t iRaw;
    int32_t dRaw;
    int32_t rawLimit;
    int32_t scaledP;
    int32_t scaledI;
    int32_t scaledD;
    int32_t totalRaw;
    bool freezeIntegral;
    bool lineValid;
    int32_t prevErrForPid;
#endif

    nearestUs = 1000.0f;
    if ((gLineFollowUsL > 0.0f) && (gLineFollowUsL < nearestUs)) {
        nearestUs = gLineFollowUsL;
    }
    if ((gLineFollowUsR > 0.0f) && (gLineFollowUsR < nearestUs)) {
        nearestUs = gLineFollowUsR;
    }
#if ROBOT_CFG_ENABLE_RAMP_FEATURE
    if ((gLineFollowRampConf == 0U) && (nearestUs < ROBOT_CFG_LINE_OBS_WARN_CM)) {
        if (baseDuty > ROBOT_CFG_LINE_DUTY_CRUISE) {
            baseDuty = ROBOT_CFG_LINE_DUTY_CRUISE;
        }
    }
#else
    if (nearestUs < ROBOT_CFG_LINE_OBS_WARN_CM) {
        if (baseDuty > ROBOT_CFG_LINE_DUTY_CRUISE) {
            baseDuty = ROBOT_CFG_LINE_DUTY_CRUISE;
        }
    }
#endif

    derr = err - gLineFollowErrPrev;
#if ENABLE_PID_DEBUG_WATCH
    gLineFollowPidLineValid = (gLineFollowCount > 0U) ? 1U : 0U;
#endif

#if ENABLE_PID_LINE_FOLLOW
    lineValid = (gLineFollowCount > 0U);
    prevErrForPid = gLineFollowErrPrev;
#if ENABLE_PID_SOFT_START_FIX
    if (lineValid && (gLineFollowPidFirstSample != 0U)) {
        prevErrForPid = err;
    }
#endif
    effectiveErr = err;
    if ((effectiveErr > -PID_DEADBAND_ERR) && (effectiveErr < PID_DEADBAND_ERR)) {
        effectiveErr = 0;
    }

    sumCandidate = gLineFollowPidSumState;
#if ENABLE_PID_INTEGRAL
    if (lineValid) {
        sumCandidate += effectiveErr;
        sumCandidate = LineFollow_ClampSigned(sumCandidate, PID_I_SUM_MAX);
    }
    iRaw = KI_LINE * sumCandidate;
#else
    iRaw = 0;
#endif
    derr = err - prevErrForPid;
    dRaw = KD_LINE * derr;
    dRaw = LineFollow_ClampSigned(dRaw, PID_D_TERM_MAX * PID_SCALE);
    pRaw = KP_LINE * effectiveErr;
    rawLimit = corrClamp;
    if (rawLimit > PID_CORR_MAX) {
        rawLimit = PID_CORR_MAX;
    }
    if (baseDuty > PID_BASE_DUTY) {
        baseDuty = PID_BASE_DUTY;
    }
    if (soften) {
        rawLimit /= 2;
    }

    totalRaw = pRaw + iRaw + dRaw;
    freezeIntegral = false;
#if ENABLE_PID_INTEGRAL
    if (totalRaw > (rawLimit * PID_SCALE)) {
        freezeIntegral = (effectiveErr > 0);
    } else if (totalRaw < -(rawLimit * PID_SCALE)) {
        freezeIntegral = (effectiveErr < 0);
    }
    if (freezeIntegral) {
        sumCandidate = gLineFollowPidSumState;
        iRaw = KI_LINE * sumCandidate;
        totalRaw = pRaw + iRaw + dRaw;
    }
#endif

    corr = -(totalRaw / PID_SCALE);
    if (soften) {
        corr /= 2;
    }
    corr = LineFollow_ClampSigned(corr, rawLimit);

    gLineFollowPidSumState = sumCandidate;
    gLineFollowPidFirstSample = 0U;
#if ENABLE_PID_DEBUG_WATCH
    gLineFollowPidErr = (int)effectiveErr;
    gLineFollowPidPrevErr = (int)prevErrForPid;
    gLineFollowPidDErr = (int)derr;
    gLineFollowPidSumErr = (int)gLineFollowPidSumState;
    gLineFollowPidBaseDuty = baseDuty;
    scaledP = -(pRaw / PID_SCALE);
    scaledI = -(iRaw / PID_SCALE);
    scaledD = -(dRaw / PID_SCALE);
    if (soften) {
        scaledP /= 2;
        scaledI /= 2;
        scaledD /= 2;
    }
    gLineFollowPidPterm = (int)scaledP;
    gLineFollowPidIterm = (int)scaledI;
    gLineFollowPidDterm = (int)scaledD;
    gLineFollowPidCorr = (int)corr;
    gLineFollowPidIntegralFrozen = freezeIntegral ? 1U : 0U;
    gLineFollowPidClamped = ((corr >= rawLimit) || (corr <= -rawLimit)) ? 1U : 0U;
#endif
#else
    corr = ((err * ROBOT_CFG_LINE_KP_NUM) + (derr * ROBOT_CFG_LINE_KD_NUM)) /
           ROBOT_CFG_LINE_ERR_DIV;

    if (soften) {
        corr /= 2;
    }

    if (corr > corrClamp) {
        corr = corrClamp;
    } else if (corr < -corrClamp) {
        corr = -corrClamp;
    }

#if ENABLE_PID_DEBUG_WATCH
    gLineFollowPidErr = (int)err;
    gLineFollowPidPrevErr = gLineFollowErrPrev;
    gLineFollowPidDErr = (int)derr;
    gLineFollowPidSumErr = 0;
    gLineFollowPidBaseDuty = baseDuty;
    gLineFollowPidPterm = corr;
    gLineFollowPidIterm = 0;
    gLineFollowPidDterm = 0;
    gLineFollowPidCorr = (int)corr;
    gLineFollowPidIntegralFrozen = 0U;
    gLineFollowPidClamped = ((corr >= corrClamp) || (corr <= -corrClamp)) ? 1U : 0U;
#endif
#endif

    leftDuty = (int32_t)baseDuty - corr;
    rightDuty = (int32_t)baseDuty + corr;

    gLineFollowBaseDuty = baseDuty;
    gLineFollowCorr = (int)corr;

    leftDuty = (int32_t)LineFollow_ClampDuty(leftDuty);
    rightDuty = (int32_t)LineFollow_ClampDuty(rightDuty);
#if ENABLE_PID_DEBUG_WATCH
    gLineFollowPidLeftDuty = (unsigned int)leftDuty;
    gLineFollowPidRightDuty = (unsigned int)rightDuty;
#endif

    if ((leftDuty <= (int32_t)ROBOT_CFG_LINE_DUTY_MIN) ||
        (rightDuty <= (int32_t)ROBOT_CFG_LINE_DUTY_MIN)) {
        gLineFollowSatTimer += ROBOT_CFG_MAIN_CONTROL_LOOP_MS;
    } else {
        gLineFollowSatTimer = 0U;
    }

    LineFollow_CommandForward((uint16_t)leftDuty, (uint16_t)rightDuty);
}

#if ROBOT_CFG_ENABLE_RAMP_FEATURE
static void LineFollow_UpdateRampTraverse(uint8_t count, uint8_t longestRun, int32_t err)
{
    int32_t absErr;
    uint16_t rampBaseDuty;
    int32_t rampCorrClamp;

    absErr = (err < 0) ? -err : err;

    if (gRampDirection >= 0) {
        gLineFollowIsRampUp = 1U;
        gLineFollowIsRampDown = 0U;
        rampBaseDuty = LineFollow_ScaleDuty(ROBOT_CFG_LINE_DUTY_SLOW,
                                            ROBOT_CFG_RAMP_FACTOR_PCT);
        rampCorrClamp = ROBOT_CFG_LINE_CORR_CLAMP_NORMAL;
    } else {
        gLineFollowIsRampUp = 0U;
        gLineFollowIsRampDown = 1U;
        rampBaseDuty = LineFollow_ScaleDuty(ROBOT_CFG_LINE_DUTY_SLOW,
                                            ROBOT_CFG_RAMP_DOWN_FACTOR_PCT);
        rampCorrClamp = ROBOT_CFG_RAMP_DOWN_CORR_CLAMP;
    }

    if ((count == 0U) && (gLineFollowLastLineSide == 0)) {
        gLineFollowLastLineSide = (gRampDirection >= 0) ? 1 : -1;
    }

    if ((gRampDirection > 0) &&
        (gLineFollowPitchDeg < ROBOT_CFG_RAMP_PITCH_EXIT_DEG) &&
        LineFollow_HasNormalLineStats(count, longestRun, absErr)) {
        gRampExitTicks++;
    } else if ((gRampDirection < 0) &&
               (gLineFollowPitchDeg > -ROBOT_CFG_RAMP_PITCH_EXIT_DEG) &&
               LineFollow_HasNormalLineStats(count, longestRun, absErr)) {
        gRampExitTicks++;
    } else {
        gRampExitTicks = 0U;
    }

    gLineFollowRampExitCounter = gRampExitTicks;

    if (gRampExitTicks >= ROBOT_CFG_RAMP_EXIT_N) {
        gLineFollowMode = LINE_FOLLOW_MODE_FOLLOW_LINE;
        gLineFollowRampConf = 0U;
        gRampDirection = 0;
        gRampExitTicks = 0U;
        gLineFollowIsRampUp = 0U;
        gLineFollowIsRampDown = 0U;
        return;
    }

    if (count > 0U) {
        LineFollow_ApplyPd(err, rampBaseDuty, rampCorrClamp, false);
    } else {
        gLineFollowMode = LINE_FOLLOW_MODE_FOLLOW_LINE;
        gLineFollowRampConf = 0U;
        gRampDirection = 0;
        LineFollow_UpdateFollowState(gLineFollowMask, count, err, longestRun);
    }
}
#endif

static void LineFollow_StartRecoverSearch(int side)
{
    LineFollow_ResetPid(3U);
    gLineFollowMode = LINE_FOLLOW_MODE_RECOVER_SEARCH;
#if ROBOT_CFG_ENABLE_RAMP_FEATURE
    gRampDirection = 0;
    gRampExitTicks = 0U;
    gLineFollowRampConf = 0U;
    gLineFollowIsRampUp = 0U;
    gLineFollowIsRampDown = 0U;
#endif
    LineFollowMode_RecoveryStart(&gLineRecoverCtx, side);
    gLineFollowSweepPhase = (unsigned int)gLineRecoverCtx.state;
    gLineFollowRecoverState = (unsigned int)gLineRecoverCtx.state;
    gLineFollowRecoverCycles = 0U;
    gGapTimerCycles = 0U;
    gLineFollowGapTimer = 0U;
}

static void LineFollow_UpdateRecoverSearch(uint8_t count, int32_t err)
{
    LineFollowRecoveryStep step;

    LineFollow_ResetPid(4U);
    LineFollowMode_RecoveryStep(&gLineRecoverCtx, count, err, &step);
    gLineFollowSweepPhase = (unsigned int)step.state;
    gLineFollowRecoverState = (unsigned int)step.state;
    gLineFollowRecoverCycles = (unsigned int)step.totalCycles;
    LineFollow_ApplyRecoveryCommand(&step);

    if (step.reacquired != 0U) {
        gLineFollowMode = LINE_FOLLOW_MODE_FOLLOW_LINE;
        gLineFollowSweepPhase = 0U;
        gLineFollowRecoverState = LINE_RECOVERY_IDLE;
        gLineFollowRecoverCycles = 0U;
        return;
    }

    if (step.timedOut != 0U) {
        gLineFollowMode = LINE_FOLLOW_MODE_FAILSAFE_STOP;
        gLineFollowSweepPhase = (unsigned int)LINE_RECOVERY_TIMEOUT_STOP;
        gLineFollowRecoverState = (unsigned int)LINE_RECOVERY_TIMEOUT_STOP;
        return;
    }
}

static void LineFollow_UpdateFollowState(uint8_t mask, uint8_t count, int32_t err, uint8_t longestRun)
{
    int32_t absErr;
    int32_t absDerr;
    bool isCentered;
    bool isGentle;
    bool isSharp;
    bool lowCurvature;

    if (gLineFollowRecoverState != LINE_RECOVERY_IDLE) {
        gLineFollowRecoverState = LINE_RECOVERY_IDLE;
        gLineFollowRecoverCycles = 0U;
    }

    absErr = (err < 0) ? -err : err;
    absDerr = err - gLineFollowErrPrev;
    if (absDerr < 0) {
        absDerr = -absDerr;
    }
    isCentered = (count >= 1U) && (count <= 3U) && (absErr < ROBOT_CFG_LINE_E_CENTER);
    isGentle = (count >= 2U) && (count <= 5U) &&
               (absErr >= ROBOT_CFG_LINE_E_CENTER) &&
               (absErr < ROBOT_CFG_LINE_E_SHARP);
    isSharp = (count > 0U) && (absErr >= ROBOT_CFG_LINE_E_SHARP);
    lowCurvature = (absDerr < (ROBOT_CFG_LINE_E_CENTER / 2));

    if (isCentered) {
        gCenterTicks++;
    } else {
        gCenterTicks = 0U;
    }

    if (isGentle) {
        gGentleTicks++;
    } else {
        gGentleTicks = 0U;
    }

    if (isSharp) {
        gSharpTicks++;
    } else {
        gSharpTicks = 0U;
        gLineFollowSatTimer = 0U;
    }

    if (isCentered && lowCurvature) {
        gGapEligibleTicks++;
    } else {
        gGapEligibleTicks = 0U;
    }

#if ROBOT_CFG_ENABLE_RAMP_FEATURE
    if ((gRampUpTicks >= ROBOT_CFG_RAMP_N) && (count > 0U)) {
        LineFollow_ResetPid(5U);
        gLineFollowMode = LINE_FOLLOW_MODE_RAMP_TRAVERSE;
        gRampDirection = 1;
        gRampExitTicks = 0U;
        gLineFollowRampConf = gRampUpTicks;
        return;
    }

    if ((gRampDownTicks >= ROBOT_CFG_RAMP_N) && (count > 0U)) {
        LineFollow_ResetPid(6U);
        gLineFollowMode = LINE_FOLLOW_MODE_RAMP_TRAVERSE;
        gRampDirection = -1;
        gRampExitTicks = 0U;
        gLineFollowRampConf = gRampDownTicks;
        return;
    }
#endif

#if ENABLE_STARTUP_LINE_ACQUIRE_FIX
    if ((gLineFollowStartupLineLatched != 0U) ||
        (gLineFollowStartupSettleRemainingMs > 0U)) {
        if (count > 0U) {
            gLineFollowStartupLineLatched = 0U;
            LineFollow_ApplyStartupHold();
        } else {
            LineFollow_CommandStop();
        }

        if (gLineFollowStartupSettleRemainingMs > ROBOT_CFG_MAIN_CONTROL_LOOP_MS) {
            gLineFollowStartupSettleRemainingMs -= ROBOT_CFG_MAIN_CONTROL_LOOP_MS;
        } else {
            gLineFollowStartupSettleRemainingMs = 0U;
        }
        return;
    }
#endif

    if (LineFollow_IsFullWidthPattern(count, longestRun)) {
        gLineFollowLostConfirmCount = 0U;
        gLineFollowCornerAssistCycles = 0U;
        LineFollow_ResetPid(11U);
        LineFollow_ApplyStartupHold();
        return;
    }

    if ((count == 1U) && (absErr >= ROBOT_CFG_LINE_E_SHARP)) {
        gLineFollowLostConfirmCount = 0U;
        gLineFollowCornerAssistCycles = 0U;
        LineFollow_ResetPid(12U);
        if (err < 0) {
            LineFollow_CommandLeft(ROBOT_CFG_LINE_CORNER_PIVOT_DUTY,
                                   ROBOT_CFG_LINE_CORNER_PIVOT_DUTY);
        } else {
            LineFollow_CommandRight(ROBOT_CFG_LINE_CORNER_PIVOT_DUTY,
                                    ROBOT_CFG_LINE_CORNER_PIVOT_DUTY);
        }
        return;
    }

    if ((count == 0U) && (gGapEligibleTicks >= 3U)) {
        gLineFollowGapConf++;
    } else if (count > 0U) {
        gLineFollowGapConf = 0U;
    }

    if ((count == 0U) && (gLineFollowGapConf >= ROBOT_CFG_LINE_GAP_N)) {
        LineFollow_ResetPid(9U);
        gLineFollowMode = LINE_FOLLOW_MODE_GAP_BRIDGE;
        gGapTimerCycles = 0U;
        gLineFollowGapTimer = 0U;
        LineFollow_CommandForward(ROBOT_CFG_LINE_DUTY_CRUISE, ROBOT_CFG_LINE_DUTY_CRUISE);
        return;
    }

    if (count == 0U) {
        if ((gLastAbsErr >= ROBOT_CFG_LINE_E_SHARP) &&
            (gLineFollowLastLineSide != 0) &&
            (gLineFollowCornerAssistCycles < ROBOT_CFG_LINE_CORNER_ASSIST_CYCLES)) {
            gLineFollowCornerAssistCycles++;
            gLineFollowLostConfirmCount = 0U;
            LineFollow_ResetPid(13U);
            if (gLineFollowLastLineSide < 0) {
                LineFollow_CommandLeft(ROBOT_CFG_LINE_CORNER_PIVOT_DUTY,
                                       ROBOT_CFG_LINE_CORNER_PIVOT_DUTY);
            } else {
                LineFollow_CommandRight(ROBOT_CFG_LINE_CORNER_PIVOT_DUTY,
                                        ROBOT_CFG_LINE_CORNER_PIVOT_DUTY);
            }
            return;
        }

        gLineFollowCornerAssistCycles = 0U;
        if (gLineFollowLostConfirmCount < ROBOT_CFG_LINE_LOST_CONFIRM_N) {
            gLineFollowLostConfirmCount++;
        }
    } else {
        gLineFollowLostConfirmCount = 0U;
        gLineFollowCornerAssistCycles = 0U;
    }

    if (gLineFollowLostConfirmCount >= ROBOT_CFG_LINE_LOST_CONFIRM_N) {
        if (LineFollow_IsExtremeLeft(mask)) {
            gLineFollowLastLineSide = -1;
        } else if (LineFollow_IsExtremeRight(mask)) {
            gLineFollowLastLineSide = 1;
        }
        LineFollow_StartRecoverSearch(gLineFollowLastLineSide);
        return;
    }

    if (gSharpTicks >= ROBOT_CFG_LINE_SHARP_N) {
        LineFollow_ApplyPd(err, ROBOT_CFG_LINE_DUTY_SLOW,
                           ROBOT_CFG_LINE_CORR_CLAMP_SHARP, false);
        if ((gLineFollowSatTimer >=
             (ROBOT_CFG_LINE_SAT_CYCLES * ROBOT_CFG_MAIN_CONTROL_LOOP_MS)) &&
            (absErr >= (gLastAbsErr - 50))) {
            LineFollow_StartRecoverSearch(gLineFollowLastLineSide);
        }
        return;
    }

    if (gGentleTicks >= ROBOT_CFG_LINE_GENTLE_N) {
        LineFollow_ApplyPd(err, ROBOT_CFG_LINE_DUTY_CRUISE,
                           ROBOT_CFG_LINE_CORR_CLAMP_NORMAL, false);
        return;
    }

    if (gCenterTicks >= ROBOT_CFG_LINE_N_CENTER) {
        LineFollow_ApplyPd(err, ROBOT_CFG_LINE_DUTY_FAST,
                           ROBOT_CFG_LINE_CORR_CLAMP_NORMAL, false);
        return;
    }

    if (count > 0U) {
        gLineFollowLostConfirmCount = 0U;
        gLineFollowCornerAssistCycles = 0U;
        LineFollow_ApplyPd(err, ROBOT_CFG_LINE_DUTY_CRUISE,
                           ROBOT_CFG_LINE_CORR_CLAMP_NORMAL, false);
    } else {
        gLineFollowCornerAssistCycles = 0U;
        LineFollow_ResetPid(10U);
        LineFollow_CommandStop();
    }
}
