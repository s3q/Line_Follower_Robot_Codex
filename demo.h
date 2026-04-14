#ifndef DEMO_H_
#define DEMO_H_

#include <stdint.h>

/*
 * Demo mode stages for the integrated end-to-end verification flow.
 */
#define DEMO_STAGE_STARTUP                      0U
#define DEMO_STAGE_SENSOR_CHECK                 1U
#define DEMO_STAGE_LINE_FOLLOW                  2U
#define DEMO_STAGE_AVOID_CHECK                  3U
#define DEMO_STAGE_COLLISION_CHECK              4U
#define DEMO_STAGE_EMERGENCY_CHECK              5U

/*
 * Demo/debug watch variables.
 * These are kept global so they are easy to inspect in CCS watch.
 */
extern volatile uint8_t gDemoStage;
extern volatile uint8_t gDemoBumpState;
extern volatile uint8_t gDemoCollisionZone;
extern volatile uint8_t gDemoLineData;
extern volatile float gDemoUltraLeftCm;
extern volatile float gDemoUltraRightCm;
extern volatile uint16_t gDemoIrRaw;
extern volatile uint8_t gDemoEmergencyActive;

void Demo_Init(void);
void Demo_Update(void);

#endif /* DEMO_H_ */
