#include "system_test.h"
#include "demo.h"

volatile uint8_t gSystemTestStage = SYSTEM_TEST_STAGE_STARTUP;
volatile uint8_t gSystemTestBumpMask = 0U;
volatile uint8_t gSystemTestCollisionZone = 0U;
volatile uint8_t gSystemTestLineData = 0U;
volatile uint16_t gSystemTestFrontIrRaw = 0U;
volatile float gSystemTestUltraLeftCm = -1.0f;
volatile float gSystemTestUltraRightCm = -1.0f;
volatile uint8_t gSystemTestEmergencyActive = 0U;

static void SystemTest_SyncFromDemo(void);

void SystemTest_Init(void)
{
    Demo_Init();
    SystemTest_SyncFromDemo();
}

void SystemTest_Update(void)
{
    Demo_Update();
    SystemTest_SyncFromDemo();
}

static void SystemTest_SyncFromDemo(void)
{
    gSystemTestStage = gDemoStage;
    gSystemTestBumpMask = gDemoBumpState;
    gSystemTestCollisionZone = gDemoCollisionZone;
    gSystemTestLineData = gDemoLineData;
    gSystemTestFrontIrRaw = gDemoIrRaw;
    gSystemTestUltraLeftCm = gDemoUltraLeftCm;
    gSystemTestUltraRightCm = gDemoUltraRightCm;
    gSystemTestEmergencyActive = gDemoEmergencyActive;
}
