#ifndef LINE_FOLLOW_H_
#define LINE_FOLLOW_H_

typedef enum {
    LINE_FOLLOW_MODE_FOLLOW_LINE = 0,
    LINE_FOLLOW_MODE_INTERSECTION_LOCK = 1, // if u know the number of intersection use this 
    LINE_FOLLOW_MODE_WIDE_LINE_STABILISE = 2,
    LINE_FOLLOW_MODE_GAP_BRIDGE = 3,
    LINE_FOLLOW_MODE_RAMP_TRAVERSE = 4,
    LINE_FOLLOW_MODE_RECOVER_SEARCH = 5,
    LINE_FOLLOW_MODE_FAILSAFE_STOP = 6
} LineFollowMode;

extern volatile LineFollowMode gLineFollowMode;
extern volatile unsigned int gLineFollowMask;
extern volatile unsigned int gLineFollowCount;
extern volatile int gLineFollowErr;
extern volatile int gLineFollowErrPrev;
extern volatile unsigned int gLineFollowBaseDuty;
extern volatile int gLineFollowCorr;
extern volatile float gLineFollowUsL;
extern volatile float gLineFollowUsR;
extern volatile unsigned int gLineFollowIrRaw;
extern volatile unsigned int gLineFollowIrMm;
extern volatile int gLineFollowPitchDeg;
extern volatile int gLineFollowPitchInputDeg;
extern volatile unsigned int gLineFollowRampConf;
extern volatile unsigned int gLineFollowObsConf;
extern volatile unsigned int gLineFollowSymmetryMetric;
extern volatile int gLineFollowLastLineSide;
extern volatile unsigned int gLineFollowSweepPhase;
extern volatile unsigned int gLineFollowSatTimer;
extern volatile unsigned int gLineFollowGapTimer;
extern volatile unsigned int gLineFollowGapConf;
extern volatile unsigned int gLineFollowIntersectConf;
extern volatile unsigned int gLineFollowLockRemainingMs;
extern volatile unsigned int gLineFollowMaskContiguousScore;
extern volatile unsigned int gLineFollowAvgReflect;
extern volatile unsigned int gLineFollowIsRampUp;
extern volatile unsigned int gLineFollowIsRampDown;
extern volatile unsigned int gLineFollowRampEnterCounter;
extern volatile unsigned int gLineFollowRampExitCounter;
extern volatile int gLineFollowPidErr;
extern volatile int gLineFollowPidPrevErr;
extern volatile int gLineFollowPidDErr;
extern volatile int gLineFollowPidSumErr;
extern volatile int gLineFollowPidCorr;
extern volatile unsigned int gLineFollowPidBaseDuty;
extern volatile unsigned int gLineFollowPidLeftDuty;
extern volatile unsigned int gLineFollowPidRightDuty;
extern volatile int gLineFollowPidPterm;
extern volatile int gLineFollowPidIterm;
extern volatile int gLineFollowPidDterm;
extern volatile unsigned int gLineFollowPidClamped;
extern volatile unsigned int gLineFollowPidLineValid;
extern volatile unsigned int gLineFollowPidEnabled;
extern volatile unsigned int gLineFollowPidIntegralFrozen;
extern volatile unsigned int gLineFollowPidResetReason;
extern volatile unsigned int gLineFollowLineValidAtStartup;
extern volatile unsigned int gLineFollowStartupAcquireCount;
extern volatile unsigned int gLineFollowStartupLineLatched;
extern volatile unsigned int gLineFollowPidFirstSample;
extern volatile unsigned int gLineFollowCurrentState;
extern volatile unsigned int gLineFollowActive;
extern volatile unsigned int gLineFollowCmdMotion;
extern volatile unsigned int gLineFollowCmdLeftDuty;
extern volatile unsigned int gLineFollowCmdRightDuty;
extern volatile unsigned int gLineFollowRecoverState;
extern volatile unsigned int gLineFollowRecoverCycles;

void LineFollow_Init(void);
void LineFollow_Update(void);

#endif /* LINE_FOLLOW_H_ */
