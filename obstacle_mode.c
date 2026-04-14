#include "obstacle_mode.h"
#include "robot_config.h"
#include <string.h>

#define OBSTACLE_TURN_LEFT         1U
#define OBSTACLE_TURN_RIGHT        2U

static uint16_t Obstacle_ClampU16(uint16_t value, uint16_t minValue, uint16_t maxValue);
static int16_t Obstacle_ClampS16(int16_t value, int16_t minValue, int16_t maxValue);
static int16_t Obstacle_WrapAngleDeg(int16_t angle);
static uint16_t Obstacle_MsToCycles(uint16_t ms);
static uint16_t Obstacle_ScoreDistance(uint16_t value, uint16_t warn, uint16_t stop);
static uint16_t Obstacle_ComputeFrontHazard(const ObstacleModeInput *in,
                                            uint16_t *leftHazard,
                                            uint16_t *rightHazard);
static uint8_t Obstacle_SelectTurnDir(const ObstacleModeInput *in,
                                      uint16_t leftHazard,
                                      uint16_t rightHazard);
static uint8_t Obstacle_IsLineRecovered(const ObstacleModeInput *in);
static void Obstacle_SetMotion(ObstacleModeOutput *out,
                               ObstacleMotion motion,
                               uint16_t leftDuty,
                               uint16_t rightDuty);
static void Obstacle_ApplyYawHold(ObstacleModeContext *ctx,
                                  const ObstacleModeInput *in,
                                  ObstacleModeOutput *out);

void ObstacleMode_Init(ObstacleModeContext *ctx)
{
    if (ctx == 0) {
        return;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->state = OBSTACLE_MODE_IDLE;
    ctx->turnDir = OBSTACLE_TURN_RIGHT;
}

void ObstacleMode_Update(ObstacleModeContext *ctx,
                         const ObstacleModeInput *in,
                         ObstacleModeOutput *out)
{
    uint16_t frontHazard;
    uint16_t leftHazard;
    uint16_t rightHazard;
    uint16_t backoffCycles;
    uint16_t turnCycles;
    uint16_t bypassMinCycles;
    uint16_t searchPivotCycles;
    uint16_t searchForwardCycles;
    uint16_t noLineCruiseCycles;
    uint16_t totalTimeoutCycles;
    uint8_t bumpPressed;
    uint8_t lineRecovered;

    if ((ctx == 0) || (in == 0) || (out == 0)) {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->state = ctx->state;
    out->turnDir = ctx->turnDir;

    backoffCycles = Obstacle_MsToCycles(ROBOT_CFG_BACKOFF_MS);
    turnCycles = Obstacle_MsToCycles(ROBOT_CFG_PIVOT_MS);
    bypassMinCycles = Obstacle_MsToCycles(ROBOT_CFG_MIN_BYPASS_FORWARD_MS);
    searchPivotCycles = Obstacle_MsToCycles(ROBOT_CFG_REACQUIRE_SWEEP_STEP_MS);
    searchForwardCycles = Obstacle_MsToCycles(ROBOT_CFG_REACQUIRE_FORWARD_STEP_MS);
    noLineCruiseCycles = Obstacle_MsToCycles(ROBOT_CFG_REACQUIRE_PRIMARY_TIMEOUT_MS);
    totalTimeoutCycles = Obstacle_MsToCycles(ROBOT_CFG_AVOID_TIMEOUT_MS);

    frontHazard = Obstacle_ComputeFrontHazard(in, &leftHazard, &rightHazard);
    bumpPressed = (in->bumpMask != 0U) ? 1U : 0U;
    lineRecovered = Obstacle_IsLineRecovered(in);

    out->frontHazard = frontHazard;
    out->leftHazard = leftHazard;
    out->rightHazard = rightHazard;

    if (frontHazard > ctx->maxFrontHazard) {
        ctx->maxFrontHazard = frontHazard;
    }

    if ((frontHazard >= 900U) && (bumpPressed == 0U)) {
        if (ctx->nearMissCount < 0xFFFFU) {
            ctx->nearMissCount++;
        }
    }

    if (ctx->state != OBSTACLE_MODE_IDLE) {
        out->overrideActive = 1U;
        if (ctx->stateCycles < 0xFFFFU) {
            ctx->stateCycles++;
        }
        if (ctx->totalCycles < 0xFFFFU) {
            ctx->totalCycles++;
        }
    }

    if ((ctx->state != OBSTACLE_MODE_IDLE) && (bumpPressed != 0U) &&
        (ctx->state != OBSTACLE_MODE_EMERGENCY_BACKOFF)) {
        if (ctx->bumpCount < 0xFFFFU) {
            ctx->bumpCount++;
        }
        ctx->state = OBSTACLE_MODE_EMERGENCY_BACKOFF;
        ctx->stateCycles = 0U;
        ctx->confirmCycles = 0U;
        ctx->clearCycles = 0U;
    }

    if ((ctx->state != OBSTACLE_MODE_IDLE) &&
        (ctx->totalCycles >= totalTimeoutCycles) &&
        (ctx->state != OBSTACLE_MODE_TIMEOUT_STOP)) {
        ctx->state = OBSTACLE_MODE_TIMEOUT_STOP;
        ctx->stateCycles = 0U;
        if (ctx->timeoutCount < 0xFFFFU) {
            ctx->timeoutCount++;
        }
    }

    switch (ctx->state) {
    case OBSTACLE_MODE_IDLE:
        out->overrideActive = 0U;
        out->motion = OBSTACLE_MOTION_STOP;
        if (bumpPressed != 0U) {
            if (ctx->bumpCount < 0xFFFFU) {
                ctx->bumpCount++;
            }
            ctx->state = OBSTACLE_MODE_EMERGENCY_BACKOFF;
            ctx->stateCycles = 0U;
            ctx->totalCycles = 0U;
            ctx->confirmCycles = 0U;
            ctx->clearCycles = 0U;
            out->overrideActive = 1U;
        } else if (frontHazard >= 500U) {
            ctx->state = OBSTACLE_MODE_CONFIRM;
            ctx->stateCycles = 0U;
            ctx->totalCycles = 0U;
            ctx->confirmCycles = 1U;
            ctx->clearCycles = 0U;
            out->overrideActive = 1U;
        }
        break;

    case OBSTACLE_MODE_CONFIRM:
        out->overrideActive = 1U;
        if (frontHazard >= 500U) {
            if (ctx->confirmCycles < 0xFFFFU) {
                ctx->confirmCycles++;
            }
            ctx->clearCycles = 0U;
        } else {
            if (ctx->clearCycles < 0xFFFFU) {
                ctx->clearCycles++;
            }
            ctx->confirmCycles = 0U;
        }

        Obstacle_SetMotion(out, OBSTACLE_MOTION_FORWARD,
                           ROBOT_CFG_LINE_DUTY_SLOW,
                           ROBOT_CFG_LINE_DUTY_SLOW);

        if (ctx->clearCycles >= 2U) {
            ctx->state = OBSTACLE_MODE_IDLE;
            ctx->stateCycles = 0U;
            ctx->totalCycles = 0U;
            out->overrideActive = 0U;
            Obstacle_SetMotion(out, OBSTACLE_MOTION_STOP, 0U, 0U);
        } else if (ctx->confirmCycles >= ROBOT_CFG_OBS_CONFIRM_N) {
            ctx->turnDir = Obstacle_SelectTurnDir(in, leftHazard, rightHazard);
            if (ctx->attemptCount < 0xFFFFU) {
                ctx->attemptCount++;
            }
            if (in->yawValid != 0U) {
                ctx->yawTargetDeg = in->yawDeg;
            }
            ctx->state = OBSTACLE_MODE_TURN_OUT;
            ctx->stateCycles = 0U;
            ctx->confirmCycles = 0U;
            ctx->clearCycles = 0U;
        }
        break;

    case OBSTACLE_MODE_EMERGENCY_BACKOFF:
        out->overrideActive = 1U;
        Obstacle_SetMotion(out, OBSTACLE_MOTION_BACKWARD,
                           ROBOT_CFG_DUTY_BACKOFF,
                           ROBOT_CFG_DUTY_BACKOFF);
        if (ctx->stateCycles >= backoffCycles) {
            ctx->turnDir = Obstacle_SelectTurnDir(in, leftHazard, rightHazard);
            if (in->yawValid != 0U) {
                ctx->yawTargetDeg = in->yawDeg;
            }
            ctx->state = OBSTACLE_MODE_TURN_OUT;
            ctx->stateCycles = 0U;
        }
        break;

    case OBSTACLE_MODE_TURN_OUT:
        out->overrideActive = 1U;
        if (ctx->turnDir == OBSTACLE_TURN_LEFT) {
            Obstacle_SetMotion(out, OBSTACLE_MOTION_LEFT,
                               ROBOT_CFG_AVOID_TURN_LEFT_DUTY,
                               ROBOT_CFG_AVOID_TURN_RIGHT_DUTY);
        } else {
            Obstacle_SetMotion(out, OBSTACLE_MOTION_RIGHT,
                               ROBOT_CFG_AVOID_TURN_LEFT_DUTY,
                               ROBOT_CFG_AVOID_TURN_RIGHT_DUTY);
        }
        if (ctx->stateCycles >= turnCycles) {
            ctx->state = OBSTACLE_MODE_BYPASS_FORWARD;
            ctx->stateCycles = 0U;
            ctx->clearCycles = 0U;
            ctx->reacquireCycles = 0U;
            if (in->yawValid != 0U) {
                ctx->yawTargetDeg = in->yawDeg;
            }
        }
        break;

    case OBSTACLE_MODE_BYPASS_FORWARD:
        out->overrideActive = 1U;
        Obstacle_SetMotion(out, OBSTACLE_MOTION_FORWARD,
                           ROBOT_CFG_LINE_DUTY_SLOW,
                           ROBOT_CFG_LINE_DUTY_SLOW);
        Obstacle_ApplyYawHold(ctx, in, out);

        if (frontHazard < 250U) {
            if (ctx->clearCycles < 0xFFFFU) {
                ctx->clearCycles++;
            }
        } else {
            ctx->clearCycles = 0U;
        }

        if ((ctx->clearCycles >= ROBOT_CFG_OBS_CONFIRM_N) &&
            (ctx->stateCycles >= bypassMinCycles)) {
            ctx->state = OBSTACLE_MODE_SEARCH_LINE;
            ctx->stateCycles = 0U;
            ctx->reacquireCycles = 0U;
            ctx->searchPhase = 0U;
        }
        break;

    case OBSTACLE_MODE_SEARCH_LINE:
        out->overrideActive = 1U;
        if (lineRecovered != 0U) {
            if (ctx->reacquireCycles < 0xFFFFU) {
                ctx->reacquireCycles++;
            }
        } else {
            ctx->reacquireCycles = 0U;
        }

        if (ctx->reacquireCycles >= ROBOT_CFG_REACQUIRE_N) {
            ctx->state = OBSTACLE_MODE_EXIT_CONFIRM;
            ctx->stateCycles = 0U;
            ctx->reacquireCycles = 0U;
            out->lineRecovered = 1U;
            break;
        }

        if (ctx->searchPhase == 0U) {
            if (ctx->turnDir == OBSTACLE_TURN_LEFT) {
                Obstacle_SetMotion(out, OBSTACLE_MOTION_RIGHT,
                                   ROBOT_CFG_LINE_DUTY_PIVOT,
                                   ROBOT_CFG_LINE_DUTY_PIVOT);
            } else {
                Obstacle_SetMotion(out, OBSTACLE_MOTION_LEFT,
                                   ROBOT_CFG_LINE_DUTY_PIVOT,
                                   ROBOT_CFG_LINE_DUTY_PIVOT);
            }
            if (ctx->stateCycles >= searchPivotCycles) {
                ctx->searchPhase = 1U;
                ctx->stateCycles = 0U;
            }
        } else if (ctx->searchPhase == 1U) {
            if (ctx->turnDir == OBSTACLE_TURN_LEFT) {
                Obstacle_SetMotion(out, OBSTACLE_MOTION_LEFT,
                                   ROBOT_CFG_LINE_DUTY_PIVOT,
                                   ROBOT_CFG_LINE_DUTY_PIVOT);
            } else {
                Obstacle_SetMotion(out, OBSTACLE_MOTION_RIGHT,
                                   ROBOT_CFG_LINE_DUTY_PIVOT,
                                   ROBOT_CFG_LINE_DUTY_PIVOT);
            }
            if (ctx->stateCycles >= (searchPivotCycles * 2U)) {
                ctx->searchPhase = 2U;
                ctx->stateCycles = 0U;
            }
        } else {
            Obstacle_SetMotion(out, OBSTACLE_MOTION_FORWARD,
                               ROBOT_CFG_LINE_DUTY_SLOW,
                               ROBOT_CFG_LINE_DUTY_SLOW);
            if (ctx->stateCycles >= searchForwardCycles) {
                ctx->state = OBSTACLE_MODE_NO_LINE_CRUISE;
                ctx->stateCycles = 0U;
                ctx->reacquireCycles = 0U;
                if (in->yawValid != 0U) {
                    ctx->yawTargetDeg = in->yawDeg;
                }
            }
        }
        break;

    case OBSTACLE_MODE_EXIT_CONFIRM:
        out->overrideActive = 1U;
        Obstacle_SetMotion(out, OBSTACLE_MOTION_FORWARD,
                           ROBOT_CFG_LINE_DUTY_SLOW,
                           ROBOT_CFG_LINE_DUTY_SLOW);
        if (lineRecovered != 0U) {
            if (ctx->reacquireCycles < 0xFFFFU) {
                ctx->reacquireCycles++;
            }
        } else {
            ctx->state = OBSTACLE_MODE_SEARCH_LINE;
            ctx->stateCycles = 0U;
            ctx->searchPhase = 0U;
            ctx->reacquireCycles = 0U;
            break;
        }

        if (ctx->reacquireCycles >= ROBOT_CFG_REACQUIRE_N) {
            if (ctx->successCount < 0xFFFFU) {
                ctx->successCount++;
            }
            ctx->state = OBSTACLE_MODE_IDLE;
            ctx->stateCycles = 0U;
            ctx->totalCycles = 0U;
            ctx->reacquireCycles = 0U;
            out->lineRecovered = 1U;
            out->overrideActive = 0U;
            Obstacle_SetMotion(out, OBSTACLE_MOTION_STOP, 0U, 0U);
        }
        break;

    case OBSTACLE_MODE_NO_LINE_CRUISE:
        out->overrideActive = 1U;
        Obstacle_SetMotion(out, OBSTACLE_MOTION_FORWARD,
                           ROBOT_CFG_LINE_DUTY_SLOW,
                           ROBOT_CFG_LINE_DUTY_SLOW);
        Obstacle_ApplyYawHold(ctx, in, out);

        if (lineRecovered != 0U) {
            ctx->state = OBSTACLE_MODE_EXIT_CONFIRM;
            ctx->stateCycles = 0U;
            ctx->reacquireCycles = 1U;
            break;
        }

        if (frontHazard >= 600U) {
            ctx->state = OBSTACLE_MODE_CONFIRM;
            ctx->stateCycles = 0U;
            ctx->confirmCycles = 1U;
            ctx->clearCycles = 0U;
            break;
        }

        if (ctx->stateCycles >= noLineCruiseCycles) {
            ctx->state = OBSTACLE_MODE_TIMEOUT_STOP;
            ctx->stateCycles = 0U;
            if (ctx->timeoutCount < 0xFFFFU) {
                ctx->timeoutCount++;
            }
        }
        break;

    case OBSTACLE_MODE_TIMEOUT_STOP:
    default:
        out->overrideActive = 1U;
        out->timedOut = 1U;
        Obstacle_SetMotion(out, OBSTACLE_MOTION_STOP, 0U, 0U);
        if (lineRecovered != 0U) {
            ctx->state = OBSTACLE_MODE_EXIT_CONFIRM;
            ctx->stateCycles = 0U;
            ctx->reacquireCycles = 1U;
        }
        break;
    }

    out->state = ctx->state;
    out->turnDir = ctx->turnDir;
    out->stateCycles = ctx->stateCycles;
    out->totalCycles = ctx->totalCycles;
}

static uint16_t Obstacle_ClampU16(uint16_t value, uint16_t minValue, uint16_t maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

static int16_t Obstacle_ClampS16(int16_t value, int16_t minValue, int16_t maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

static int16_t Obstacle_WrapAngleDeg(int16_t angle)
{
    while (angle > 180) {
        angle -= 360;
    }
    while (angle < -180) {
        angle += 360;
    }
    return angle;
}

static uint16_t Obstacle_MsToCycles(uint16_t ms)
{
    uint32_t c = ((uint32_t)ms + (ROBOT_CFG_MAIN_CONTROL_LOOP_MS - 1U)) /
                 ROBOT_CFG_MAIN_CONTROL_LOOP_MS;
    if (c == 0U) {
        c = 1U;
    }
    if (c > 0xFFFFU) {
        c = 0xFFFFU;
    }
    return (uint16_t)c;
}

static uint16_t Obstacle_ScoreDistance(uint16_t value, uint16_t warn, uint16_t stop)
{
    uint32_t score;
    uint32_t num;
    uint32_t den;

    if (value == 0U) {
        return 0U;
    }
    if (value <= stop) {
        return 1000U;
    }
    if (value >= warn) {
        return 0U;
    }

    num = ((uint32_t)warn - value) * 1000U;
    den = (uint32_t)warn - stop;
    score = num / den;
    if (score > 1000U) {
        score = 1000U;
    }
    return (uint16_t)score;
}

static uint16_t Obstacle_ComputeFrontHazard(const ObstacleModeInput *in,
                                            uint16_t *leftHazard,
                                            uint16_t *rightHazard)
{
    uint16_t usLeftHaz;
    uint16_t usRightHaz;
    uint16_t usFrontHaz;
    uint16_t irHaz;
    uint16_t bumpHaz;
    uint16_t fused;

    usLeftHaz = Obstacle_ScoreDistance(in->usLeftCm,
                                       ROBOT_CFG_OBS_WARN_CM,
                                       ROBOT_CFG_OBS_STOP_CM);
    usRightHaz = Obstacle_ScoreDistance(in->usRightCm,
                                        ROBOT_CFG_OBS_WARN_CM,
                                        ROBOT_CFG_OBS_STOP_CM);
    irHaz = Obstacle_ScoreDistance(in->irMm,
                                   ROBOT_CFG_IR_WARN_MM,
                                   ROBOT_CFG_IR_STOP_MM);

    if ((in->rampLikely != 0U) && (irHaz < 500U) && (in->bumpMask == 0U)) {
        usLeftHaz = 0U;
        usRightHaz = 0U;
    }

    usFrontHaz = (usLeftHaz > usRightHaz) ? usLeftHaz : usRightHaz;
    fused = (uint16_t)(((uint32_t)irHaz * 6U + (uint32_t)usFrontHaz * 4U) / 10U);
    bumpHaz = (in->bumpMask != 0U) ? 1000U : 0U;
    if (bumpHaz > fused) {
        fused = bumpHaz;
    }

    if ((in->bumpMask & 0x07U) != 0U) {
        usLeftHaz = Obstacle_ClampU16((uint16_t)(usLeftHaz + 400U), 0U, 1000U);
    }
    if ((in->bumpMask & 0x38U) != 0U) {
        usRightHaz = Obstacle_ClampU16((uint16_t)(usRightHaz + 400U), 0U, 1000U);
    }

    *leftHazard = usLeftHaz;
    *rightHazard = usRightHaz;
    return fused;
}

static uint8_t Obstacle_SelectTurnDir(const ObstacleModeInput *in,
                                      uint16_t leftHazard,
                                      uint16_t rightHazard)
{
    (void)in;
    if (leftHazard <= rightHazard) {
        return OBSTACLE_TURN_LEFT;
    }
    return OBSTACLE_TURN_RIGHT;
}

static uint8_t Obstacle_IsLineRecovered(const ObstacleModeInput *in)
{
    return (in->lineCount > 0U) &&
           (in->lineErr > -ROBOT_CFG_REACQUIRE_E_ACCEPT) &&
           (in->lineErr < ROBOT_CFG_REACQUIRE_E_ACCEPT);
}

static void Obstacle_SetMotion(ObstacleModeOutput *out,
                               ObstacleMotion motion,
                               uint16_t leftDuty,
                               uint16_t rightDuty)
{
    out->motion = motion;
    out->leftDuty = leftDuty;
    out->rightDuty = rightDuty;
}

static void Obstacle_ApplyYawHold(ObstacleModeContext *ctx,
                                  const ObstacleModeInput *in,
                                  ObstacleModeOutput *out)
{
    int16_t yawErr;
    int16_t corr;
    int32_t left;
    int32_t right;

    if ((in->yawValid == 0U) || (out->motion != OBSTACLE_MOTION_FORWARD)) {
        return;
    }

    yawErr = Obstacle_WrapAngleDeg((int16_t)(in->yawDeg - ctx->yawTargetDeg));
    corr = Obstacle_ClampS16((int16_t)(yawErr / 6), -18, 18);

    left = (int32_t)out->leftDuty - corr;
    right = (int32_t)out->rightDuty + corr;

    if (left < (int32_t)ROBOT_CFG_LINE_DUTY_MIN) {
        left = ROBOT_CFG_LINE_DUTY_MIN;
    }
    if (right < (int32_t)ROBOT_CFG_LINE_DUTY_MIN) {
        right = ROBOT_CFG_LINE_DUTY_MIN;
    }
    if (left > (int32_t)ROBOT_CFG_LINE_DUTY_FAST) {
        left = ROBOT_CFG_LINE_DUTY_FAST;
    }
    if (right > (int32_t)ROBOT_CFG_LINE_DUTY_FAST) {
        right = ROBOT_CFG_LINE_DUTY_FAST;
    }

    out->leftDuty = (uint16_t)left;
    out->rightDuty = (uint16_t)right;
}
