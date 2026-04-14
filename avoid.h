#ifndef AVOID_H_
#define AVOID_H_

#include <stdbool.h>

typedef enum {
    AVOID_STATE_IDLE = 0,
    AVOID_STATE_OBSTACLE_APPROACH_CONFIRM = 1,
    AVOID_STATE_EMERGENCY_BUMP = 2,
    AVOID_STATE_OBSTACLE_BYPASS = 3,
    AVOID_STATE_LINE_REACQUIRE_GATED = 4
} AvoidState;

typedef enum {
    AVOID_BYPASS_SIDE_NONE = 0,
    AVOID_BYPASS_SIDE_LEFT = 1,
    AVOID_BYPASS_SIDE_RIGHT = 2
} AvoidBypassSide;

extern volatile AvoidState gAvoidState;
extern volatile unsigned int gAvoidObsConf;
extern volatile unsigned int gAvoidObsConfirmCount;
extern volatile unsigned int gAvoidObstacleWarnCount;
extern volatile unsigned int gAvoidObstacleClearCount;
extern volatile unsigned int gAvoidIrMm;
extern volatile unsigned int gAvoidSymmetryMetricCm;
extern volatile unsigned int gAvoidBumpMask6;
extern volatile unsigned int gAvoidBumpZone;
extern volatile unsigned int gAvoidChosenBypassSide;
extern volatile unsigned int gAvoidBackoffRemainingMs;
extern volatile unsigned int gAvoidRepeatedBumpCount;
extern volatile unsigned int gAvoidReacquireEnabled;
extern volatile unsigned int gAvoidReacquireStableCount;
extern volatile unsigned int gAvoidBypassPhase;
extern volatile unsigned int gAvoidBypassTimerMs;
extern volatile unsigned int gAvoidMinUsCm;
extern volatile unsigned int gAvoidObstacleConfirmed;
extern volatile unsigned int gAvoidObstacleSuppressedByRamp;
extern volatile float gAvoidUsLeftCm;
extern volatile float gAvoidUsRightCm;
extern volatile unsigned int gAvoidAiObstacleValid;
extern volatile unsigned int gAvoidAiBypassSide;
extern volatile unsigned int gAvoidAiObstacleClass;
extern volatile unsigned int gAvoidAiConfidence;
extern volatile unsigned int gAvoidAiDataAgeTicks;
extern volatile unsigned int gAvoidAiFresh;
extern volatile unsigned int gAvoidStep;
extern volatile unsigned int gAvoidStepTimerMs;
extern volatile unsigned int gAvoidClearExtendCounter;
extern volatile int gAvoidSideErr;
extern volatile int gAvoidPrevSideErr;
extern volatile int gAvoidCorrSide;
extern volatile unsigned int gAvoidBaseDuty;
extern volatile unsigned int gAvoidBoundaryModeActive;
extern volatile unsigned int gAvoidFrontBlocked;
extern volatile unsigned int gAvoidAvoidTimerMs;
extern volatile int gAvoidYawInputDeg;
extern volatile unsigned int gAvoidYawInputFresh;
extern volatile int gAvoidEntryYaw;
extern volatile int gAvoidYawNow;
extern volatile int gAvoidYawDelta;
extern volatile unsigned int gAvoidLineAcceptRejectReason;
extern volatile unsigned int gAvoidReacquireAcceptCount;
extern volatile unsigned int gAvoidReacquireSweepStage;
extern volatile unsigned int gAvoidReacquireStepTimerMs;
extern volatile unsigned int gAvoidLineCount;
extern volatile int gAvoidLineErr;
extern volatile unsigned int gAvoidFrontMinRangeCm;
extern volatile unsigned int gAvoidReacquireGatePassed;
extern volatile unsigned int gAvoidHeadingSource;
extern volatile unsigned int gAvoidNearMissCount;
extern volatile unsigned int gAvoidBumpEventCount;
extern volatile unsigned int gAvoidAttemptCount;
extern volatile unsigned int gAvoidSuccessCount;
extern volatile unsigned int gAvoidTimeoutCount;

void Avoid_Init(void);
bool Avoid_ShouldOverride(void);
void Avoid_Update(void);

#endif /* AVOID_H_ */
