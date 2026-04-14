#include "demo.h"
#include "bump.h"
#include "collision.h"
#include "control.h"
#include "ir.h"
#include "line.h"
#include "motor.h"
#include "robot_config.h"
#include "robot_pins.h"
#include "ultrasonic.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

volatile uint8_t gDemoStage = DEMO_STAGE_STARTUP;
volatile uint8_t gDemoBumpState = 0U;
volatile uint8_t gDemoCollisionZone = COLLISION_NONE;
volatile uint8_t gDemoLineData = 0U;
volatile float gDemoUltraLeftCm = -1.0f;
volatile float gDemoUltraRightCm = -1.0f;
volatile uint16_t gDemoIrRaw = 0U;
volatile uint8_t gDemoEmergencyActive = 0U;

static uint32_t gDemoStateCycles = 0U;

static void Demo_SetStageLeds(uint8_t stage);
static void Demo_AdvanceStageIfNeeded(void);

void Demo_Init(void)
{
    /*
     * Demo mode is the integrated staged verification path for the robot.
     * It checks sensors first, then allows the normal control stack to run
     * through staged observation windows. Emergency still has highest priority
     * because all behavior-capable stages use Control_Update().
     */
    Bump_Init();
    Line_Init();
    Ultrasonic_Init();
    IR_Init();

    GPIO_setAsOutputPin(LED_GREEN_PORT, LED_GREEN_PIN);
    GPIO_setAsOutputPin(LED_BLUE_PORT, LED_BLUE_PIN);
    GPIO_setOutputLowOnPin(LED_GREEN_PORT, LED_GREEN_PIN);
    GPIO_setOutputLowOnPin(LED_BLUE_PORT, LED_BLUE_PIN);

    Motor_Stop();
    gDemoStage = DEMO_STAGE_STARTUP;
    gDemoStateCycles = 0U;
    gDemoEmergencyActive = 0U;
    Demo_SetStageLeds(gDemoStage);
}

void Demo_Update(void)
{
    gDemoBumpState = Bump_Read();
    gDemoCollisionZone = Collision_FromBumpMask(gDemoBumpState);
    gDemoLineData = Line_Read(ROBOT_CFG_LINE_FOLLOW_SAMPLE_US);
    gDemoUltraLeftCm = Ultrasonic_ReadLeftCm();
    gDemoUltraRightCm = Ultrasonic_ReadRightCm();
    gDemoIrRaw = IR_ReadRaw();

    switch (gDemoStage) {
    case DEMO_STAGE_STARTUP:
    case DEMO_STAGE_SENSOR_CHECK:
        /*
         * Startup and sensor verification are non-motion stages.
         * This keeps the first part of the demo safe while you confirm:
         * - bump bitmask changes
         * - line data changes
         * - front IR / ultrasonic values are readable
         */
        Motor_Stop();
        break;

    case DEMO_STAGE_LINE_FOLLOW:
    case DEMO_STAGE_AVOID_CHECK:
    case DEMO_STAGE_COLLISION_CHECK:
    case DEMO_STAGE_EMERGENCY_CHECK:
    default:
        /*
         * These stages validate integrated behavior, not a separate demo motor
         * script. The existing control stack remains responsible for priority:
         * collision/emergency first, avoidance second, line following last.
         */
        Control_Update();
        break;
    }

    gDemoEmergencyActive = (gControlState == CONTROL_STATE_EMERGENCY) ? 1U : 0U;
    Demo_AdvanceStageIfNeeded();
    Demo_SetStageLeds(gDemoStage);
}

static void Demo_SetStageLeds(uint8_t stage)
{
    switch (stage) {
    case DEMO_STAGE_STARTUP:
        GPIO_setOutputHighOnPin(LED_GREEN_PORT, LED_GREEN_PIN);
        GPIO_setOutputLowOnPin(LED_BLUE_PORT, LED_BLUE_PIN);
        break;
    case DEMO_STAGE_SENSOR_CHECK:
        GPIO_toggleOutputOnPin(LED_GREEN_PORT, LED_GREEN_PIN);
        GPIO_setOutputLowOnPin(LED_BLUE_PORT, LED_BLUE_PIN);
        break;
    case DEMO_STAGE_LINE_FOLLOW:
        GPIO_setOutputHighOnPin(LED_GREEN_PORT, LED_GREEN_PIN);
        GPIO_setOutputLowOnPin(LED_BLUE_PORT, LED_BLUE_PIN);
        break;
    case DEMO_STAGE_AVOID_CHECK:
        GPIO_setOutputLowOnPin(LED_GREEN_PORT, LED_GREEN_PIN);
        GPIO_setOutputHighOnPin(LED_BLUE_PORT, LED_BLUE_PIN);
        break;
    case DEMO_STAGE_COLLISION_CHECK:
        GPIO_toggleOutputOnPin(LED_BLUE_PORT, LED_BLUE_PIN);
        GPIO_setOutputHighOnPin(LED_GREEN_PORT, LED_GREEN_PIN);
        break;
    case DEMO_STAGE_EMERGENCY_CHECK:
    default:
        GPIO_setOutputHighOnPin(LED_GREEN_PORT, LED_GREEN_PIN);
        GPIO_setOutputHighOnPin(LED_BLUE_PORT, LED_BLUE_PIN);
        break;
    }
}

static void Demo_AdvanceStageIfNeeded(void)
{
    gDemoStateCycles++;

    switch (gDemoStage) {
    case DEMO_STAGE_STARTUP:
        if (gDemoStateCycles >= ROBOT_CFG_DEMO_STARTUP_CYCLES) {
            gDemoStage = DEMO_STAGE_SENSOR_CHECK;
            gDemoStateCycles = 0U;
        }
        break;
    case DEMO_STAGE_SENSOR_CHECK:
        if (gDemoStateCycles >= ROBOT_CFG_DEMO_SENSOR_STAGE_CYCLES) {
            gDemoStage = DEMO_STAGE_LINE_FOLLOW;
            gDemoStateCycles = 0U;
        }
        break;
    case DEMO_STAGE_LINE_FOLLOW:
        if (gDemoStateCycles >= ROBOT_CFG_DEMO_LINE_STAGE_CYCLES) {
            gDemoStage = DEMO_STAGE_AVOID_CHECK;
            gDemoStateCycles = 0U;
        }
        break;
    case DEMO_STAGE_AVOID_CHECK:
        if (gDemoStateCycles >= ROBOT_CFG_DEMO_AVOID_STAGE_CYCLES) {
            gDemoStage = DEMO_STAGE_COLLISION_CHECK;
            gDemoStateCycles = 0U;
        }
        break;
    case DEMO_STAGE_COLLISION_CHECK:
        if (gDemoStateCycles >= ROBOT_CFG_DEMO_COLLISION_STAGE_CYCLES) {
            gDemoStage = DEMO_STAGE_EMERGENCY_CHECK;
            gDemoStateCycles = 0U;
        }
        break;
    case DEMO_STAGE_EMERGENCY_CHECK:
    default:
        break;
    }
}
