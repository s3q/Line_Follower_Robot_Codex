#ifndef SYSTEM_TEST_H_
#define SYSTEM_TEST_H_

#include "demo.h"

/*
 * Legacy compatibility wrapper:
 * the staged end-to-end test flow now lives in demo.c, but this header is
 * kept so stale CCS project entries that still reference system_test.* do not
 * break the build.
 */
#define SYSTEM_TEST_STAGE_STARTUP                 DEMO_STAGE_STARTUP
#define SYSTEM_TEST_STAGE_SENSORS                 DEMO_STAGE_SENSOR_CHECK
#define SYSTEM_TEST_STAGE_LINE_FOLLOW             DEMO_STAGE_LINE_FOLLOW
#define SYSTEM_TEST_STAGE_AVOID                   DEMO_STAGE_AVOID_CHECK
#define SYSTEM_TEST_STAGE_COLLISION               DEMO_STAGE_COLLISION_CHECK
#define SYSTEM_TEST_STAGE_EMERGENCY               DEMO_STAGE_EMERGENCY_CHECK

extern volatile uint8_t gSystemTestStage;
extern volatile uint8_t gSystemTestBumpMask;
extern volatile uint8_t gSystemTestCollisionZone;
extern volatile uint8_t gSystemTestLineData;
extern volatile uint16_t gSystemTestFrontIrRaw;
extern volatile float gSystemTestUltraLeftCm;
extern volatile float gSystemTestUltraRightCm;
extern volatile uint8_t gSystemTestEmergencyActive;

void SystemTest_Init(void);
void SystemTest_Update(void);

#endif /* SYSTEM_TEST_H_ */
