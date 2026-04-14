#ifndef OBSTACLE_MODE_H_
#define OBSTACLE_MODE_H_

#include <stdint.h>

typedef enum {
    OBSTACLE_MODE_IDLE = 0,
    OBSTACLE_MODE_CONFIRM = 1,
    OBSTACLE_MODE_EMERGENCY_BACKOFF = 2,
    OBSTACLE_MODE_TURN_OUT = 3,
    OBSTACLE_MODE_BYPASS_FORWARD = 4,
    OBSTACLE_MODE_SEARCH_LINE = 5,
    OBSTACLE_MODE_EXIT_CONFIRM = 6,
    OBSTACLE_MODE_NO_LINE_CRUISE = 7,
    OBSTACLE_MODE_TIMEOUT_STOP = 8
} ObstacleModeState;

typedef enum {
    OBSTACLE_MOTION_STOP = 0,
    OBSTACLE_MOTION_FORWARD = 1,
    OBSTACLE_MOTION_LEFT = 2,
    OBSTACLE_MOTION_RIGHT = 3,
    OBSTACLE_MOTION_BACKWARD = 4
} ObstacleMotion;

typedef struct {
    uint16_t irMm;
    uint16_t usLeftCm;
    uint16_t usRightCm;
    uint8_t bumpMask;
    uint8_t lineCount;
    int16_t lineErr;
    int16_t yawDeg;
    uint8_t yawValid;
    uint8_t rampLikely;
} ObstacleModeInput;

typedef struct {
    ObstacleModeState state;
    ObstacleMotion motion;
    uint16_t leftDuty;
    uint16_t rightDuty;
    uint16_t frontHazard;
    uint16_t leftHazard;
    uint16_t rightHazard;
    uint8_t turnDir;          /* 1 = left, 2 = right */
    uint8_t overrideActive;   /* 1 while obstacle mode owns motor commands */
    uint8_t lineRecovered;    /* pulse when line reacquire is confirmed */
    uint8_t timedOut;         /* 1 when state reaches timeout stop */
    uint16_t stateCycles;
    uint16_t totalCycles;
} ObstacleModeOutput;

typedef struct {
    ObstacleModeState state;
    uint16_t stateCycles;
    uint16_t totalCycles;
    uint16_t confirmCycles;
    uint16_t clearCycles;
    uint16_t reacquireCycles;
    uint8_t searchPhase;
    uint8_t turnDir;          /* 1 = left, 2 = right */
    int16_t yawTargetDeg;
    uint16_t nearMissCount;
    uint16_t bumpCount;
    uint16_t attemptCount;
    uint16_t successCount;
    uint16_t timeoutCount;
    uint16_t maxFrontHazard;
} ObstacleModeContext;

void ObstacleMode_Init(ObstacleModeContext *ctx);
void ObstacleMode_Update(ObstacleModeContext *ctx,
                         const ObstacleModeInput *in,
                         ObstacleModeOutput *out);

#endif /* OBSTACLE_MODE_H_ */
