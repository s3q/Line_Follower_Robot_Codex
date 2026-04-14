#ifndef MODE_MANAGER_H_
#define MODE_MANAGER_H_

#include <stdint.h>
#include "control.h"
#include "zone_detector.h"

extern volatile unsigned int gModeManagerSequenceProgress;
extern volatile unsigned int gModeManagerLockoutTicks;
extern volatile unsigned int gModeManagerAcceptedSwitches;
extern volatile unsigned int gModeManagerRejectedSwitches;

void ModeManager_Init(void);
void ModeManager_Tick(void);
ControlState ModeManager_BootMode(void);
uint8_t ModeManager_IsModeEnabled(ControlState mode);
ControlState ModeManager_ZoneToMode(ZoneId zone);
ControlState ModeManager_ApplyProfileRequest(ControlState current, ControlState requested);
uint32_t ModeManager_ModeTimeoutTicks(ControlState mode);
ControlState ModeManager_SequenceStepMode(uint8_t stepIndex);
ControlState ModeManager_DecideMode(
    ControlState current,
    const ZoneDecision *zoneDecision,
    uint8_t obstacleTrigger,
    uint8_t torqueTrigger,
    uint8_t forceStop,
    uint32_t modeTicks);

#endif /* MODE_MANAGER_H_ */
