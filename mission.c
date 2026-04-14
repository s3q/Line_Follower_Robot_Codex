#include "mission.h"
#include "avoid.h"
#include "bump.h"
#include "collision.h"
#include "emergency.h"
#include "line.h"
#include "line_follow.h"
#include "mission_config.h"
#include "mode_manager.h"
#include "motor.h"
#include "reaction.h"
#include "zone_detector.h"

volatile ControlState gMissionMode = CONTROL_STATE_STOP;
volatile unsigned int gMissionModeTicks = 0U;
volatile unsigned int gMissionWatchdogTripped = 0U;
volatile unsigned int gMissionTimeoutStop = 0U;
volatile unsigned int gMissionZoneSwitches = 0U;
volatile unsigned int gMissionTorqueDuty = 0U;
volatile unsigned int gMissionTorquePhase = MISSION_TORQUE_PHASE_ENTRY;
volatile unsigned int gMissionTorqueRampDetected = 0U;
volatile int gMissionTorquePitchDeg = 0;
volatile int gMissionTorqueYawDeg = 0;
volatile int gMissionTorqueYawTargetDeg = 0;
volatile int gMissionTorqueYawErr = 0;
volatile int gMissionTorqueYawCorr = 0;
volatile unsigned int gMissionTorqueLineCount = 0U;
volatile unsigned int gMissionTorqueFault = 0U;
volatile unsigned int gMissionTorqueLeftDuty = 0U;
volatile unsigned int gMissionTorqueRightDuty = 0U;

static unsigned int gMissionInitDone = 0U;
static unsigned int gMissionInitWatchdogTicks = 0U;
static unsigned int gMissionProgressTicks = 0U;
static unsigned int gMissionTorquePhaseTicks = 0U;
static unsigned int gMissionTorquePitchEnterCount = 0U;
static unsigned int gMissionTorquePitchExitCount = 0U;
static unsigned int gMissionTorqueLineConfirmCount = 0U;
static unsigned int gMissionTorqueExitTicks = 0U;
static unsigned int gMissionTorqueYawTargetValid = 0U;
static int gMissionTorqueRampSign = 0;
static unsigned int gMissionFallbackObsConfirmCount = 0U;
static unsigned int gMissionFallbackTorqueConfirmCount = 0U;

static int Mission_AbsI(int value);
static int Mission_WrapAngleDeg(int angleDeg);
static uint8_t Mission_CountLineBits(uint8_t mask);
static uint16_t Mission_ClampDuty(int32_t duty);
static void Mission_SetTorquePhase(MissionTorquePhase phase);
static void Mission_EnterMode(ControlState nextMode);
static void Mission_UpdateLineFollow(void);
static void Mission_UpdateObstacleAvoid(void);
static void Mission_UpdateTorqueRamp(void);
static void Mission_UpdateStop(void);

static int Mission_AbsI(int value)
{
    return (value < 0) ? -value : value;
}

static int Mission_WrapAngleDeg(int angleDeg)
{
    while (angleDeg > 180) {
        angleDeg -= 360;
    }
    while (angleDeg < -180) {
        angleDeg += 360;
    }
    return angleDeg;
}

static uint8_t Mission_CountLineBits(uint8_t mask)
{
    uint8_t count = 0U;

    while (mask != 0U) {
        count += (uint8_t)(mask & 1U);
        mask >>= 1;
    }

    return count;
}

static uint16_t Mission_ClampDuty(int32_t duty)
{
    if (duty < (int32_t)ROBOT_CFG_LINE_DUTY_MIN) {
        return ROBOT_CFG_LINE_DUTY_MIN;
    }
    if (duty > (int32_t)ROBOT_CFG_LINE_DUTY_FAST) {
        return ROBOT_CFG_LINE_DUTY_FAST;
    }
    return (uint16_t)duty;
}

static void Mission_SetTorquePhase(MissionTorquePhase phase)
{
    gMissionTorquePhase = (unsigned int)phase;
    gMissionTorquePhaseTicks = 0U;
}

static void Mission_EnterMode(ControlState nextMode)
{
    gMissionMode = nextMode;
    gMissionModeTicks = 0U;
    gMissionProgressTicks = 0U;

    if (nextMode == CONTROL_STATE_TORQUE_RAMP) {
        gMissionTorqueDuty = MISSION_TORQUE_START_DUTY;
        gMissionTorquePhaseTicks = 0U;
        gMissionTorquePitchEnterCount = 0U;
        gMissionTorquePitchExitCount = 0U;
        gMissionTorqueLineConfirmCount = 0U;
        gMissionTorqueExitTicks = 0U;
        gMissionTorqueYawTargetValid = 0U;
        gMissionTorqueRampSign = 0;
        gMissionTorqueRampDetected = 0U;
        gMissionTorquePitchDeg = 0;
        gMissionTorqueYawDeg = 0;
        gMissionTorqueYawTargetDeg = 0;
        gMissionTorqueYawErr = 0;
        gMissionTorqueYawCorr = 0;
        gMissionTorqueLineCount = 0U;
        gMissionTorqueFault = 0U;
        gMissionTorqueLeftDuty = 0U;
        gMissionTorqueRightDuty = 0U;
        Mission_SetTorquePhase(MISSION_TORQUE_PHASE_ENTRY);
    }

    if (nextMode == CONTROL_STATE_STOP) {
        gMissionTorqueLeftDuty = 0U;
        gMissionTorqueRightDuty = 0U;
        Motor_Stop();
    }
}

static void Mission_UpdateLineFollow(void)
{
#if MISSION_EN_LINE_FOLLOW
    LineFollow_Update();
    if (gLineFollowCount > 0U) {
        gMissionProgressTicks = 0U;
    }
#else
    Mission_EnterMode(CONTROL_STATE_STOP);
#endif
}

static void Mission_UpdateObstacleAvoid(void)
{
#if MISSION_EN_OBSTACLE_AVOID
    if (Avoid_ShouldOverride()) {
        Avoid_Update();
        gMissionProgressTicks = 0U;
    } else if (MISSION_EN_LINE_FOLLOW) {
        LineFollow_Update();
        if (gLineFollowCount > 0U) {
            gMissionProgressTicks = 0U;
        }
    } else {
        Motor_Forward(ROBOT_CFG_LINE_DUTY_SLOW, ROBOT_CFG_LINE_DUTY_SLOW);
    }
#else
    Mission_EnterMode(CONTROL_STATE_STOP);
#endif
}

static void Mission_UpdateTorqueRamp(void)
{
#if MISSION_EN_TORQUE_RAMP
    uint8_t lineMask;
    uint8_t lineCount;
    uint8_t bumpMask;
    int pitchDeg;
    int absPitch;
    int yawDeg;
    int yawErr;
    int yawCorr;
    uint16_t baseDuty;
    uint16_t leftDuty;
    uint16_t rightDuty;

    if (gMissionTorquePhaseTicks < 0xFFFFFFFFU) {
        gMissionTorquePhaseTicks++;
    }
    if (gMissionTorqueExitTicks < 0xFFFFFFFFU) {
        gMissionTorqueExitTicks++;
    }

    lineMask = Line_Read(ROBOT_CFG_LINE_FOLLOW_SAMPLE_US);
    lineCount = Mission_CountLineBits(lineMask);
    bumpMask = Bump_Read();
    pitchDeg = gLineFollowPitchInputDeg;
    absPitch = Mission_AbsI(pitchDeg);
    gMissionTorquePitchDeg = pitchDeg;
    gMissionTorqueLineCount = lineCount;

    if (gAvoidYawInputFresh != 0U) {
        yawDeg = Mission_WrapAngleDeg(gAvoidYawInputDeg);
        gMissionTorqueYawDeg = yawDeg;
        if (gMissionTorqueYawTargetValid == 0U) {
            gMissionTorqueYawTargetDeg = yawDeg;
            gMissionTorqueYawTargetValid = 1U;
        }
    } else {
        yawDeg = gMissionTorqueYawDeg;
    }

    if (bumpMask != 0U) {
        gMissionTorqueFault = 3U;
        Mission_SetTorquePhase(MISSION_TORQUE_PHASE_FAULT_STOP);
        Motor_Stop();
        Mission_EnterMode(CONTROL_STATE_STOP);
        return;
    }

    if (absPitch >= MISSION_TORQUE_PITCH_ABORT_DEG) {
        gMissionTorqueFault = 1U;
        Mission_SetTorquePhase(MISSION_TORQUE_PHASE_FAULT_STOP);
        Motor_Stop();
        Mission_EnterMode(CONTROL_STATE_STOP);
        return;
    }

    baseDuty = (uint16_t)MISSION_TORQUE_START_DUTY;

    switch ((MissionTorquePhase)gMissionTorquePhase) {
    case MISSION_TORQUE_PHASE_ENTRY:
        baseDuty = (uint16_t)MISSION_TORQUE_START_DUTY;

        if (absPitch >= MISSION_TORQUE_PITCH_ENTER_DEG) {
            if (gMissionTorquePitchEnterCount < 0xFFFFFFFFU) {
                gMissionTorquePitchEnterCount++;
            }
        } else {
            gMissionTorquePitchEnterCount = 0U;
        }

        if (gMissionTorquePitchEnterCount >= MISSION_TORQUE_ENTER_CONFIRM_TICKS) {
            gMissionTorqueRampDetected = 1U;
            gMissionTorqueRampSign = (pitchDeg >= 0) ? 1 : -1;
            gMissionTorquePitchEnterCount = 0U;
            gMissionTorquePitchExitCount = 0U;
            gMissionTorqueLineConfirmCount = 0U;
            gMissionTorqueExitTicks = 0U;
            Mission_SetTorquePhase(MISSION_TORQUE_PHASE_CLIMB);
        }
        break;

    case MISSION_TORQUE_PHASE_CLIMB:
        if (absPitch >= (MISSION_TORQUE_PITCH_ENTER_DEG + 4)) {
            baseDuty = (uint16_t)MISSION_TORQUE_CLIMB_STEEP_DUTY;
        } else {
            baseDuty = (uint16_t)MISSION_TORQUE_CLIMB_DUTY;
        }

        if (gMissionTorqueRampSign == 0) {
            gMissionTorqueRampSign = (pitchDeg >= 0) ? 1 : -1;
        }

        if ((gMissionTorqueRampSign * pitchDeg) <= MISSION_TORQUE_PITCH_EXIT_DEG) {
            if (gMissionTorquePitchExitCount < 0xFFFFFFFFU) {
                gMissionTorquePitchExitCount++;
            }
        } else {
            gMissionTorquePitchExitCount = 0U;
        }

        if (gMissionTorquePitchExitCount >= MISSION_TORQUE_EXIT_CONFIRM_TICKS) {
            gMissionTorquePitchExitCount = 0U;
            Mission_SetTorquePhase(MISSION_TORQUE_PHASE_DESCENT);
        } else if (gMissionTorquePhaseTicks >= MISSION_TORQUE_MAX_CLIMB_TICKS) {
            gMissionTorqueFault = 2U;
            Mission_SetTorquePhase(MISSION_TORQUE_PHASE_FAULT_STOP);
            Motor_Stop();
            Mission_EnterMode(CONTROL_STATE_STOP);
            return;
        }
        break;

    case MISSION_TORQUE_PHASE_DESCENT:
        if (absPitch >= MISSION_TORQUE_PITCH_ENTER_DEG) {
            baseDuty = (uint16_t)MISSION_TORQUE_DESCENT_BRAKE_DUTY;
        } else {
            baseDuty = (uint16_t)MISSION_TORQUE_DESCENT_DUTY;
        }

        if (absPitch <= MISSION_TORQUE_PITCH_EXIT_DEG) {
            if (gMissionTorquePitchExitCount < 0xFFFFFFFFU) {
                gMissionTorquePitchExitCount++;
            }
        } else {
            gMissionTorquePitchExitCount = 0U;
        }

        if (gMissionTorquePitchExitCount >= MISSION_TORQUE_EXIT_CONFIRM_TICKS) {
            gMissionTorquePitchExitCount = 0U;
            gMissionTorqueLineConfirmCount = 0U;
            gMissionTorqueExitTicks = 0U;
            Mission_SetTorquePhase(MISSION_TORQUE_PHASE_EXIT_SEARCH);
        } else if (gMissionTorquePhaseTicks >= MISSION_TORQUE_MAX_DESCENT_TICKS) {
            gMissionTorqueFault = 2U;
            Mission_SetTorquePhase(MISSION_TORQUE_PHASE_FAULT_STOP);
            Motor_Stop();
            Mission_EnterMode(CONTROL_STATE_STOP);
            return;
        }
        break;

    case MISSION_TORQUE_PHASE_EXIT_SEARCH:
        baseDuty = (uint16_t)MISSION_TORQUE_EXIT_CRUISE_DUTY;

        if (absPitch >= MISSION_TORQUE_PITCH_ENTER_DEG) {
            gMissionTorqueRampSign = (pitchDeg >= 0) ? 1 : -1;
            gMissionTorquePitchExitCount = 0U;
            Mission_SetTorquePhase(MISSION_TORQUE_PHASE_CLIMB);
            break;
        }

        if (lineCount >= MISSION_TORQUE_LINE_COUNT_MIN) {
            if (gMissionTorqueLineConfirmCount < 0xFFFFFFFFU) {
                gMissionTorqueLineConfirmCount++;
            }
        } else {
            gMissionTorqueLineConfirmCount = 0U;
        }

        if (gMissionTorqueLineConfirmCount >= MISSION_TORQUE_LINE_CONFIRM_TICKS) {
#if MISSION_EN_LINE_FOLLOW
            Mission_EnterMode(CONTROL_STATE_LINE_FOLLOW);
            return;
#else
            Mission_SetTorquePhase(MISSION_TORQUE_PHASE_COMPLETE);
#endif
        } else if (gMissionTorqueExitTicks >= MISSION_TORQUE_EXIT_SEARCH_TICKS) {
            Mission_SetTorquePhase(MISSION_TORQUE_PHASE_COMPLETE);
        }
        break;

    case MISSION_TORQUE_PHASE_COMPLETE:
        baseDuty = (uint16_t)MISSION_TORQUE_EXIT_CRUISE_DUTY;
        if (lineCount >= MISSION_TORQUE_LINE_COUNT_MIN) {
            if (gMissionTorqueLineConfirmCount < 0xFFFFFFFFU) {
                gMissionTorqueLineConfirmCount++;
            }
        } else {
            gMissionTorqueLineConfirmCount = 0U;
        }
        if (gMissionTorqueLineConfirmCount >= MISSION_TORQUE_LINE_CONFIRM_TICKS) {
#if MISSION_EN_LINE_FOLLOW
            Mission_EnterMode(CONTROL_STATE_LINE_FOLLOW);
            return;
#endif
        }
        break;

    case MISSION_TORQUE_PHASE_FAULT_STOP:
    default:
        Motor_Stop();
        return;
    }

    yawErr = 0;
    yawCorr = 0;
    if (gMissionTorqueYawTargetValid != 0U) {
        yawErr = Mission_WrapAngleDeg(yawDeg - gMissionTorqueYawTargetDeg);
        if (MISSION_TORQUE_YAW_KP_DIV > 0) {
            yawCorr = (yawErr * MISSION_TORQUE_YAW_KP_NUM) / MISSION_TORQUE_YAW_KP_DIV;
        }
        if (yawCorr > MISSION_TORQUE_YAW_CORR_MAX) {
            yawCorr = MISSION_TORQUE_YAW_CORR_MAX;
        } else if (yawCorr < -MISSION_TORQUE_YAW_CORR_MAX) {
            yawCorr = -MISSION_TORQUE_YAW_CORR_MAX;
        }
    }

    leftDuty = Mission_ClampDuty((int32_t)baseDuty - (int32_t)yawCorr);
    rightDuty = Mission_ClampDuty((int32_t)baseDuty + (int32_t)yawCorr);

    gMissionTorqueDuty = baseDuty;
    gMissionTorqueYawErr = yawErr;
    gMissionTorqueYawCorr = yawCorr;
    gMissionTorqueLeftDuty = leftDuty;
    gMissionTorqueRightDuty = rightDuty;

    Motor_Forward(leftDuty, rightDuty);
    gMissionProgressTicks = 0U;
#else
    Mission_EnterMode(CONTROL_STATE_STOP);
#endif
}

static void Mission_UpdateStop(void)
{
    Motor_Stop();
}

void Mission_Init(void)
{
    gMissionInitDone = 0U;
    gMissionInitWatchdogTicks = 0U;
    gMissionWatchdogTripped = 0U;
    gMissionTimeoutStop = 0U;
    gMissionZoneSwitches = 0U;
    gMissionModeTicks = 0U;
    gMissionProgressTicks = 0U;
    gMissionFallbackObsConfirmCount = 0U;
    gMissionFallbackTorqueConfirmCount = 0U;
    gMissionTorqueDuty = MISSION_TORQUE_START_DUTY;
    gMissionTorquePhase = MISSION_TORQUE_PHASE_ENTRY;
    gMissionTorquePhaseTicks = 0U;
    gMissionTorquePitchEnterCount = 0U;
    gMissionTorquePitchExitCount = 0U;
    gMissionTorqueLineConfirmCount = 0U;
    gMissionTorqueExitTicks = 0U;
    gMissionTorqueYawTargetValid = 0U;
    gMissionTorqueRampSign = 0;
    gMissionTorqueRampDetected = 0U;
    gMissionTorquePitchDeg = 0;
    gMissionTorqueYawDeg = 0;
    gMissionTorqueYawTargetDeg = 0;
    gMissionTorqueYawErr = 0;
    gMissionTorqueYawCorr = 0;
    gMissionTorqueLineCount = 0U;
    gMissionTorqueFault = 0U;
    gMissionTorqueLeftDuty = 0U;
    gMissionTorqueRightDuty = 0U;

    /*
     * Non-blocking init policy: no loops, only one-shot init calls.
     */
    LineFollow_Init();
    Avoid_Init();
    Collision_Init();
    Reaction_Init();
    Emergency_Init();
    ZoneDetector_Init();
    ModeManager_Init();

    Mission_EnterMode(ModeManager_BootMode());
    gMissionInitDone = 1U;
}

void Mission_Update(void)
{
    uint8_t bumpMask;
    ZoneDecision zoneDecision;
    ControlState requestedMode;
    uint32_t modeTimeout;
    uint8_t fallbackObsTrigger;
    uint8_t fallbackTorqueTrigger;
    uint8_t forceStop;
    int pitchAbs;

    if (gMissionInitDone == 0U) {
        if (gMissionInitWatchdogTicks < MISSION_INIT_TIMEOUT_TICKS) {
            gMissionInitWatchdogTicks++;
            return;
        }

        gMissionWatchdogTripped = 1U;
        Mission_EnterMode(CONTROL_STATE_STOP);
        return;
    }

    if (gMissionModeTicks < 0xFFFFFFFFU) {
        gMissionModeTicks++;
    }
    if (gMissionProgressTicks < 0xFFFFFFFFU) {
        gMissionProgressTicks++;
    }
    ModeManager_Tick();

    zoneDecision.zone = ZONE_UNKNOWN;
    zoneDecision.confidence = 0U;
    zoneDecision.switchQualified = 0U;
    fallbackObsTrigger = 0U;
    fallbackTorqueTrigger = 0U;
    forceStop = 0U;

    bumpMask = Bump_Read();
    if (bumpMask != 0U) {
        forceStop = 1U;
    }

    pitchAbs = Mission_AbsI(gLineFollowPitchInputDeg);
    if (pitchAbs >= MISSION_TORQUE_PITCH_ENTER_DEG) {
        if (gMissionFallbackTorqueConfirmCount < 0xFFFFFFFFU) {
            gMissionFallbackTorqueConfirmCount++;
        }
    } else {
        gMissionFallbackTorqueConfirmCount = 0U;
    }
    if (gMissionFallbackTorqueConfirmCount >= MISSION_FALLBACK_TORQUE_CONFIRM_TICKS) {
        fallbackTorqueTrigger = 1U;
    }

#if MISSION_EN_OBSTACLE_AVOID
    if ((gMissionMode != CONTROL_STATE_OBSTACLE_AVOID) &&
        (gMissionMode != CONTROL_STATE_TORQUE_RAMP)) {
        if (Avoid_ShouldOverride()) {
            if (gMissionFallbackObsConfirmCount < 0xFFFFFFFFU) {
                gMissionFallbackObsConfirmCount++;
            }
        } else {
            gMissionFallbackObsConfirmCount = 0U;
        }
    } else {
        gMissionFallbackObsConfirmCount = 0U;
    }
    if (gMissionFallbackObsConfirmCount >= MISSION_FALLBACK_OBS_CONFIRM_TICKS) {
        fallbackObsTrigger = 1U;
    }
#endif

#if MISSION_EN_ZONE_SWITCH
#if USE_COLOR_ZONES
    {
        uint8_t lineMask = Line_Read(MISSION_ZONE_SAMPLE_US);
        zoneDecision = ZoneDetector_Update(lineMask);
    }
#endif
#endif

    requestedMode = ModeManager_DecideMode(
        gMissionMode,
        &zoneDecision,
        fallbackObsTrigger,
        fallbackTorqueTrigger,
        forceStop,
        gMissionModeTicks);

    if (requestedMode != gMissionMode) {
        gMissionZoneSwitches++;
        Mission_EnterMode(requestedMode);
    }

    modeTimeout = ModeManager_ModeTimeoutTicks(gMissionMode);
    if (gMissionModeTicks >= modeTimeout) {
        gMissionTimeoutStop = 1U;
        Mission_EnterMode(CONTROL_STATE_STOP);
    }

    if (gMissionProgressTicks >= MISSION_WATCHDOG_NO_PROGRESS_TICKS) {
        gMissionWatchdogTripped = 1U;
        Mission_EnterMode(CONTROL_STATE_STOP);
    }

    switch (gMissionMode) {
    case CONTROL_STATE_LINE_FOLLOW:
        Mission_UpdateLineFollow();
        break;
    case CONTROL_STATE_OBSTACLE_AVOID:
        Mission_UpdateObstacleAvoid();
        break;
    case CONTROL_STATE_TORQUE_RAMP:
        Mission_UpdateTorqueRamp();
        break;
    case CONTROL_STATE_STOP:
    default:
        Mission_UpdateStop();
        break;
    }
}

ControlState Mission_GetMode(void)
{
    return gMissionMode;
}
