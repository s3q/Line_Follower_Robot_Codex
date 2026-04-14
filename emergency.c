#include "emergency.h"
#include "bump.h"
#include "motor.h"
#include "robot_pins.h"
#include <stdint.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

static bool gEmergencyActive = false;
static uint8_t gEmergencyBumpState = 0U;

void Emergency_Init(void)
{
    Bump_Init();
    GPIO_setAsOutputPin(LED_RED_PORT, LED_RED_PIN);
    gEmergencyActive = false;
    gEmergencyBumpState = 0U;
}

bool Emergency_IsActive(void)
{
    /*
     * Bumper emergency is top priority because physical contact means the robot
     * may already be in an unsafe state. Once triggered, this first version
     * stays latched until reset so no other behavior can restart the motors.
     *
     * Recovery behavior can be changed later to a timed/manual release rule if
     * needed, but latching is the simplest and safest starting point.
     */
    if (!gEmergencyActive) {
        gEmergencyBumpState = Bump_Read();
        if (gEmergencyBumpState != 0U) {
            Motor_EmergencyStop();
            gEmergencyActive = true;
        }
    }

    return gEmergencyActive;
}

void Emergency_Update(void)
{
    gEmergencyBumpState = Bump_Read();
    Motor_EmergencyStop();
    GPIO_setOutputHighOnPin(LED_RED_PORT, LED_RED_PIN);
}
