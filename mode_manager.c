#include "mode_manager.h"
#include "mission_config.h"

volatile unsigned int gModeManagerSequenceProgress = 0U;
volatile unsigned int gModeManagerLockoutTicks = 0U;
volatile unsigned int gModeManagerAcceptedSwitches = 0U;
volatile unsigned int gModeManagerRejectedSwitches = 0U;

static unsigned int ModeManager_ModeProgress(ControlState mode);
static ControlState ModeManager_FilterRequest(
    ControlState current,
    ControlState requested,
    uint32_t modeTicks,
    uint8_t strictStep);

void ModeManager_Init(void)
{
    gModeManagerSequenceProgress = 0U;
    gModeManagerLockoutTicks = 0U;
    gModeManagerAcceptedSwitches = 0U;
    gModeManagerRejectedSwitches = 0U;
}

void ModeManager_Tick(void)
{
    if (gModeManagerLockoutTicks > 0U) {
        gModeManagerLockoutTicks--;
    }
}

ControlState ModeManager_BootMode(void)
{
#if MISSION_EN_SEQUENCE
    return CONTROL_STATE_LINE_FOLLOW;
#elif MISSION_EN_ZONE_SWITCH
    return CONTROL_STATE_LINE_FOLLOW;
#elif MISSION_EN_LINE_FOLLOW
    return CONTROL_STATE_LINE_FOLLOW;
#elif MISSION_EN_OBSTACLE_AVOID
    return CONTROL_STATE_OBSTACLE_AVOID;
#elif MISSION_EN_TORQUE_RAMP
    return CONTROL_STATE_TORQUE_RAMP;
#else
    return CONTROL_STATE_STOP;
#endif
}

uint8_t ModeManager_IsModeEnabled(ControlState mode)
{
    switch (mode) {
    case CONTROL_STATE_LINE_FOLLOW:
        return MISSION_EN_LINE_FOLLOW;
    case CONTROL_STATE_OBSTACLE_AVOID:
        return MISSION_EN_OBSTACLE_AVOID;
    case CONTROL_STATE_TORQUE_RAMP:
        return MISSION_EN_TORQUE_RAMP;
    case CONTROL_STATE_STOP:
    default:
        return 1U;
    }
}

ControlState ModeManager_ZoneToMode(ZoneId zone)
{
    switch (zone) {
    case ZONE_GREEN:
        return CONTROL_STATE_LINE_FOLLOW;
    case ZONE_YELLOW:
        return CONTROL_STATE_OBSTACLE_AVOID;
    case ZONE_BLUE:
        return CONTROL_STATE_TORQUE_RAMP;
    case ZONE_RED:
        return CONTROL_STATE_STOP;
    case ZONE_UNKNOWN:
    default:
        return CONTROL_STATE_STOP;
    }
}

ControlState ModeManager_ApplyProfileRequest(ControlState current, ControlState requested)
{
    if (requested == current) {
        return current;
    }

    if (ModeManager_IsModeEnabled(requested) == 0U) {
        return current;
    }

    return requested;
}

uint32_t ModeManager_ModeTimeoutTicks(ControlState mode)
{
    switch (mode) {
    case CONTROL_STATE_LINE_FOLLOW:
        return MISSION_MODE_TIMEOUT_LINE_TICKS;
    case CONTROL_STATE_OBSTACLE_AVOID:
        return MISSION_MODE_TIMEOUT_OBS_TICKS;
    case CONTROL_STATE_TORQUE_RAMP:
        return MISSION_MODE_TIMEOUT_TORQUE_TICKS;
    case CONTROL_STATE_STOP:
    default:
        return 0xFFFFFFFFU;
    }
}

ControlState ModeManager_SequenceStepMode(uint8_t stepIndex)
{
    switch (stepIndex) {
    case 0U:
        return CONTROL_STATE_LINE_FOLLOW;
    case 1U:
        return CONTROL_STATE_OBSTACLE_AVOID;
    case 2U:
        return CONTROL_STATE_TORQUE_RAMP;
    case 3U:
    default:
        return CONTROL_STATE_STOP;
    }
}

static unsigned int ModeManager_ModeProgress(ControlState mode)
{
    switch (mode) {
    case CONTROL_STATE_LINE_FOLLOW:
        return 0U;
    case CONTROL_STATE_OBSTACLE_AVOID:
        return 1U;
    case CONTROL_STATE_TORQUE_RAMP:
        return 2U;
    case CONTROL_STATE_STOP:
    default:
        return 3U;
    }
}

static ControlState ModeManager_FilterRequest(
    ControlState current,
    ControlState requested,
    uint32_t modeTicks,
    uint8_t strictStep)
{
    unsigned int reqProgress;

    if (requested == current) {
        return current;
    }

    if (ModeManager_IsModeEnabled(requested) == 0U) {
        gModeManagerRejectedSwitches++;
        return current;
    }

    if (requested == CONTROL_STATE_STOP) {
        gModeManagerSequenceProgress = 3U;
        gModeManagerAcceptedSwitches++;
        return requested;
    }

    if (gModeManagerLockoutTicks > 0U) {
        gModeManagerRejectedSwitches++;
        return current;
    }

    if (modeTicks < MISSION_MODE_MIN_DWELL_TICKS) {
        gModeManagerRejectedSwitches++;
        return current;
    }

    reqProgress = ModeManager_ModeProgress(requested);

#if MISSION_SEQUENCE_ENFORCE
    if (strictStep != 0U) {
        if (reqProgress != (gModeManagerSequenceProgress + 1U)) {
            gModeManagerRejectedSwitches++;
            return current;
        }
        gModeManagerSequenceProgress = reqProgress;
    } else {
        if (reqProgress > gModeManagerSequenceProgress) {
            gModeManagerSequenceProgress = reqProgress;
        } else if ((requested != CONTROL_STATE_LINE_FOLLOW) &&
                   (reqProgress < gModeManagerSequenceProgress)) {
            gModeManagerRejectedSwitches++;
            return current;
        }
    }
#else
    if (reqProgress > gModeManagerSequenceProgress) {
        gModeManagerSequenceProgress = reqProgress;
    }
#endif

    gModeManagerLockoutTicks = MISSION_MODE_SWITCH_LOCKOUT_TICKS;
    gModeManagerAcceptedSwitches++;
    return requested;
}

ControlState ModeManager_DecideMode(
    ControlState current,
    const ZoneDecision *zoneDecision,
    uint8_t obstacleTrigger,
    uint8_t torqueTrigger,
    uint8_t forceStop,
    uint32_t modeTicks)
{
    ControlState requested = current;

    if (forceStop != 0U) {
        return CONTROL_STATE_STOP;
    }

#if USE_COLOR_ZONES
    if ((zoneDecision == 0) || (zoneDecision->switchQualified == 0U)) {
        return current;
    }

    requested = ModeManager_ZoneToMode(zoneDecision->zone);
    requested = ModeManager_ApplyProfileRequest(current, requested);
    return ModeManager_FilterRequest(current, requested, modeTicks, 1U);
#else
    if ((current == CONTROL_STATE_TORQUE_RAMP) && (torqueTrigger == 0U)) {
        return current;
    }

    if ((torqueTrigger != 0U) && (MISSION_EN_TORQUE_RAMP != 0U)) {
        requested = CONTROL_STATE_TORQUE_RAMP;
    } else if ((obstacleTrigger != 0U) && (MISSION_EN_OBSTACLE_AVOID != 0U)) {
        requested = CONTROL_STATE_OBSTACLE_AVOID;
    } else if (MISSION_EN_LINE_FOLLOW != 0U) {
        requested = CONTROL_STATE_LINE_FOLLOW;
    } else {
        requested = CONTROL_STATE_STOP;
    }

    requested = ModeManager_ApplyProfileRequest(current, requested);
    return ModeManager_FilterRequest(current, requested, modeTicks, 0U);
#endif
}
