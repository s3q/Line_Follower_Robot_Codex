#ifndef LINE_FOLLOW_MODE_H_
#define LINE_FOLLOW_MODE_H_

#include <stdint.h>

typedef enum {
    LINE_RECOVERY_IDLE = 0,
    LINE_RECOVERY_PIVOT_PRIMARY = 1,
    LINE_RECOVERY_REVERSE = 2,
    LINE_RECOVERY_PIVOT_SECONDARY = 3,
    LINE_RECOVERY_FORWARD_VERIFY = 4,
    LINE_RECOVERY_TIMEOUT_STOP = 5
} LineFollowRecoveryState;

typedef enum {
    LINE_RECOVERY_CMD_STOP = 0,
    LINE_RECOVERY_CMD_FORWARD = 1,
    LINE_RECOVERY_CMD_LEFT = 2,
    LINE_RECOVERY_CMD_RIGHT = 3,
    LINE_RECOVERY_CMD_BACKWARD = 4
} LineFollowRecoveryCommand;

typedef struct {
    LineFollowRecoveryState state;
    int8_t primarySide;
    uint16_t stateCycles;
    uint16_t totalCycles;
    uint8_t reacquireCount;
} LineFollowRecoveryContext;

typedef struct {
    LineFollowRecoveryState state;
    LineFollowRecoveryCommand command;
    uint16_t leftDuty;
    uint16_t rightDuty;
    uint16_t totalCycles;
    uint8_t done;
    uint8_t reacquired;
    uint8_t timedOut;
} LineFollowRecoveryStep;

int32_t LineFollowMode_ComputeLineError(uint8_t mask, uint8_t count);
void LineFollowMode_RecoveryInit(LineFollowRecoveryContext *ctx);
void LineFollowMode_RecoveryStart(LineFollowRecoveryContext *ctx, int sideHint);
void LineFollowMode_RecoveryStep(LineFollowRecoveryContext *ctx,
                                 uint8_t lineCount,
                                 int32_t lineErr,
                                 LineFollowRecoveryStep *step);

#endif /* LINE_FOLLOW_MODE_H_ */
