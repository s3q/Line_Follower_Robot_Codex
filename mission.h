#ifndef MISSION_H_
#define MISSION_H_

#include "control.h"

typedef enum {
    MISSION_TORQUE_PHASE_ENTRY = 0,
    MISSION_TORQUE_PHASE_CLIMB = 1,
    MISSION_TORQUE_PHASE_DESCENT = 2,
    MISSION_TORQUE_PHASE_EXIT_SEARCH = 3,
    MISSION_TORQUE_PHASE_COMPLETE = 4,
    MISSION_TORQUE_PHASE_FAULT_STOP = 5
} MissionTorquePhase;

extern volatile ControlState gMissionMode;
extern volatile unsigned int gMissionModeTicks;
extern volatile unsigned int gMissionWatchdogTripped;
extern volatile unsigned int gMissionTimeoutStop;
extern volatile unsigned int gMissionZoneSwitches;
extern volatile unsigned int gMissionTorqueDuty;
extern volatile unsigned int gMissionTorquePhase;
extern volatile unsigned int gMissionTorqueRampDetected;
extern volatile int gMissionTorquePitchDeg;
extern volatile int gMissionTorqueYawDeg;
extern volatile int gMissionTorqueYawTargetDeg;
extern volatile int gMissionTorqueYawErr;
extern volatile int gMissionTorqueYawCorr;
extern volatile unsigned int gMissionTorqueLineCount;
extern volatile unsigned int gMissionTorqueFault;
extern volatile unsigned int gMissionTorqueLeftDuty;
extern volatile unsigned int gMissionTorqueRightDuty;

void Mission_Init(void);
void Mission_Update(void);
ControlState Mission_GetMode(void);

#endif /* MISSION_H_ */
