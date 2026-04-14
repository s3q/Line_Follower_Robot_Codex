#include "line_follow_mode.h"
#include "robot_config.h"
#include <string.h>

static uint8_t LineFollowMode_IsReacquired(uint8_t count, int32_t err);

int32_t LineFollowMode_ComputeLineError(uint8_t mask, uint8_t count)
{
    int32_t weightedSum = 0;
    uint8_t bit;

    if (count == 0U) {
        return 0;
    }

    for (bit = 0U; bit < 8U; bit++) {
        if ((mask & (uint8_t)(1U << bit)) != 0U) {
            weightedSum += (int32_t)bit * 1000;
        }
    }

    return 3500 - (weightedSum / (int32_t)count);
}

void LineFollowMode_RecoveryInit(LineFollowRecoveryContext *ctx)
{
    if (ctx == 0) {
        return;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->state = LINE_RECOVERY_IDLE;
    ctx->primarySide = 1;
}

void LineFollowMode_RecoveryStart(LineFollowRecoveryContext *ctx, int sideHint)
{
    if (ctx == 0) {
        return;
    }

    ctx->state = LINE_RECOVERY_PIVOT_PRIMARY;
    ctx->primarySide = (sideHint < 0) ? -1 : 1;
    ctx->stateCycles = 0U;
    ctx->totalCycles = 0U;
    ctx->reacquireCount = 0U;
}

void LineFollowMode_RecoveryStep(LineFollowRecoveryContext *ctx,
                                 uint8_t lineCount,
                                 int32_t lineErr,
                                 LineFollowRecoveryStep *step)
{
    if ((ctx == 0) || (step == 0)) {
        return;
    }

    memset(step, 0, sizeof(*step));
    step->state = ctx->state;
    step->command = LINE_RECOVERY_CMD_STOP;
    step->leftDuty = 0U;
    step->rightDuty = 0U;
    step->totalCycles = ctx->totalCycles;

    if (ctx->state == LINE_RECOVERY_IDLE) {
        step->done = 1U;
        return;
    }

    if (LineFollowMode_IsReacquired(lineCount, lineErr)) {
        if (ctx->reacquireCount < 0xFFU) {
            ctx->reacquireCount++;
        }
    } else {
        ctx->reacquireCount = 0U;
    }

    if (ctx->reacquireCount >= ROBOT_CFG_LINE_RECOVER_REACQUIRE_N) {
        ctx->state = LINE_RECOVERY_IDLE;
        step->state = LINE_RECOVERY_IDLE;
        step->done = 1U;
        step->reacquired = 1U;
        return;
    }

    if (ctx->totalCycles >= ROBOT_CFG_LINE_RECOVER_MAX_CYCLES) {
        ctx->state = LINE_RECOVERY_TIMEOUT_STOP;
        step->state = LINE_RECOVERY_TIMEOUT_STOP;
        step->done = 1U;
        step->timedOut = 1U;
        return;
    }

    switch (ctx->state) {
    case LINE_RECOVERY_PIVOT_PRIMARY:
        if (ctx->primarySide < 0) {
            step->command = LINE_RECOVERY_CMD_LEFT;
        } else {
            step->command = LINE_RECOVERY_CMD_RIGHT;
        }
        step->leftDuty = ROBOT_CFG_LINE_DUTY_PIVOT;
        step->rightDuty = ROBOT_CFG_LINE_DUTY_PIVOT;
        ctx->stateCycles++;
        if (ctx->stateCycles >= ROBOT_CFG_LINE_SWEEP_CYCLES) {
            ctx->state = LINE_RECOVERY_REVERSE;
            ctx->stateCycles = 0U;
        }
        break;

    case LINE_RECOVERY_REVERSE:
        step->command = LINE_RECOVERY_CMD_BACKWARD;
        step->leftDuty = ROBOT_CFG_LINE_DUTY_SLOW;
        step->rightDuty = ROBOT_CFG_LINE_DUTY_SLOW;
        ctx->stateCycles++;
        if (ctx->stateCycles >= ROBOT_CFG_LINE_RECOVER_REVERSE_CYCLES) {
            ctx->state = LINE_RECOVERY_PIVOT_SECONDARY;
            ctx->stateCycles = 0U;
        }
        break;

    case LINE_RECOVERY_PIVOT_SECONDARY:
        if (ctx->primarySide < 0) {
            step->command = LINE_RECOVERY_CMD_RIGHT;
        } else {
            step->command = LINE_RECOVERY_CMD_LEFT;
        }
        step->leftDuty = ROBOT_CFG_LINE_DUTY_PIVOT;
        step->rightDuty = ROBOT_CFG_LINE_DUTY_PIVOT;
        ctx->stateCycles++;
        if (ctx->stateCycles >= ROBOT_CFG_LINE_SWEEP_CYCLES) {
            ctx->state = LINE_RECOVERY_FORWARD_VERIFY;
            ctx->stateCycles = 0U;
        }
        break;

    case LINE_RECOVERY_FORWARD_VERIFY:
        step->command = LINE_RECOVERY_CMD_FORWARD;
        step->leftDuty = ROBOT_CFG_LINE_DUTY_SLOW;
        step->rightDuty = ROBOT_CFG_LINE_DUTY_SLOW;
        ctx->stateCycles++;
        if (ctx->stateCycles >= ROBOT_CFG_LINE_RECOVER_VERIFY_CYCLES) {
            ctx->state = LINE_RECOVERY_TIMEOUT_STOP;
            step->state = LINE_RECOVERY_TIMEOUT_STOP;
            step->command = LINE_RECOVERY_CMD_STOP;
            step->leftDuty = 0U;
            step->rightDuty = 0U;
            step->done = 1U;
            step->timedOut = 1U;
            return;
        }
        break;

    case LINE_RECOVERY_TIMEOUT_STOP:
    default:
        step->state = LINE_RECOVERY_TIMEOUT_STOP;
        step->done = 1U;
        step->timedOut = 1U;
        step->command = LINE_RECOVERY_CMD_STOP;
        return;
    }

    if (ctx->totalCycles < 0xFFFFU) {
        ctx->totalCycles++;
    }

    step->state = ctx->state;
    step->totalCycles = ctx->totalCycles;
}

static uint8_t LineFollowMode_IsReacquired(uint8_t count, int32_t err)
{
    return (count >= 1U) &&
           (err > -ROBOT_CFG_LINE_E_RECOVER_EXIT) &&
           (err < ROBOT_CFG_LINE_E_RECOVER_EXIT);
}
