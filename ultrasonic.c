#include "ultrasonic.h"
#include "robot_config.h"
#include "robot_pins.h"
#include <stdbool.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#define ULTRA_TIMER_BASE               TIMER32_0_BASE
#define ULTRA_TIMER_MAX_COUNT          0xFFFFFFFFU
static bool gUltrasonicInitDone = false;

static void Ultrasonic_DelayUs(uint32_t us);
static float Ultrasonic_ReadOneCm(uint_fast8_t trigPort, uint_fast16_t trigPin,
                                  uint_fast8_t echoPort, uint_fast16_t echoPin);
static uint32_t Ultrasonic_TimerDelta(uint32_t startTick, uint32_t endTick);

void Ultrasonic_Init(void)
{
    GPIO_setAsOutputPin(ULTRA_LEFT_TRIG_PORT, ULTRA_LEFT_TRIG_PIN);
    GPIO_setAsOutputPin(ULTRA_RIGHT_TRIG_PORT, ULTRA_RIGHT_TRIG_PIN);
    GPIO_setAsInputPin(ULTRA_LEFT_ECHO_PORT, ULTRA_LEFT_ECHO_PIN);
    GPIO_setAsInputPin(ULTRA_RIGHT_ECHO_PORT, ULTRA_RIGHT_ECHO_PIN);

    GPIO_setOutputLowOnPin(ULTRA_LEFT_TRIG_PORT, ULTRA_LEFT_TRIG_PIN);
    GPIO_setOutputLowOnPin(ULTRA_RIGHT_TRIG_PORT, ULTRA_RIGHT_TRIG_PIN);

    /*
     * Timer32 runs continuously from MCLK so pulse width can be measured
     * without changing the project's existing SysTick timing code.
     */
    Timer32_initModule(ULTRA_TIMER_BASE,
                       TIMER32_PRESCALER_1,
                       TIMER32_32BIT,
                       TIMER32_PERIODIC_MODE);
    Timer32_setCount(ULTRA_TIMER_BASE, ULTRA_TIMER_MAX_COUNT);
    Timer32_startTimer(ULTRA_TIMER_BASE, false);

    gUltrasonicInitDone = true;
}

float Ultrasonic_ReadLeftCm(void)
{
    if (!gUltrasonicInitDone) {
        Ultrasonic_Init();
    }

    return Ultrasonic_ReadOneCm(ULTRA_LEFT_TRIG_PORT, ULTRA_LEFT_TRIG_PIN,
                                ULTRA_LEFT_ECHO_PORT, ULTRA_LEFT_ECHO_PIN);
}

float Ultrasonic_ReadRightCm(void)
{
    if (!gUltrasonicInitDone) {
        Ultrasonic_Init();
    }

    return Ultrasonic_ReadOneCm(ULTRA_RIGHT_TRIG_PORT, ULTRA_RIGHT_TRIG_PIN,
                                ULTRA_RIGHT_ECHO_PORT, ULTRA_RIGHT_ECHO_PIN);
}

static void Ultrasonic_DelayUs(uint32_t us)
{
    uint32_t mclkHz = CS_getMCLK();
    uint32_t ticksPerUs = mclkHz / 1000000U;
    uint32_t ticksNeeded;
    uint32_t elapsed;
    uint32_t startTick;
    uint32_t currentTick;

    if (ticksPerUs == 0U) {
        ticksPerUs = 3U;
    }

    ticksNeeded = us * ticksPerUs;
    elapsed = 0U;
    startTick = Timer32_getValue(ULTRA_TIMER_BASE);

    while (elapsed < ticksNeeded) {
        currentTick = Timer32_getValue(ULTRA_TIMER_BASE);
        elapsed = Ultrasonic_TimerDelta(startTick, currentTick);
    }
}

static float Ultrasonic_ReadOneCm(uint_fast8_t trigPort, uint_fast16_t trigPin,
                                  uint_fast8_t echoPort, uint_fast16_t echoPin)
{
    uint32_t mclkHz;
    uint32_t ticksPerUs;
    uint32_t startWaitTick;
    uint32_t pulseStartTick;
    uint32_t pulseEndTick;
    uint32_t elapsedTicks;
    float pulseUs;

    mclkHz = CS_getMCLK();
    ticksPerUs = mclkHz / 1000000U;
    if (ticksPerUs == 0U) {
        ticksPerUs = 3U;
    }

    /* Ensure trigger starts low, then send one 10 us pulse. */
    GPIO_setOutputLowOnPin(trigPort, trigPin);
    Ultrasonic_DelayUs(2U);
    GPIO_setOutputHighOnPin(trigPort, trigPin);
    Ultrasonic_DelayUs(ROBOT_CFG_ULTRA_TRIGGER_PULSE_US);
    GPIO_setOutputLowOnPin(trigPort, trigPin);

    /* Wait for echo pulse to start. */
    startWaitTick = Timer32_getValue(ULTRA_TIMER_BASE);
    while (GPIO_getInputPinValue(echoPort, echoPin) == 0U) {
        elapsedTicks = Ultrasonic_TimerDelta(startWaitTick, Timer32_getValue(ULTRA_TIMER_BASE));
        if (elapsedTicks > (ROBOT_CFG_ULTRA_START_TIMEOUT_US * ticksPerUs)) {
            return ROBOT_CFG_ULTRA_INVALID_CM;
        }
    }

    /* Measure echo high pulse width. */
    pulseStartTick = Timer32_getValue(ULTRA_TIMER_BASE);
    while (GPIO_getInputPinValue(echoPort, echoPin) != 0U) {
        elapsedTicks = Ultrasonic_TimerDelta(pulseStartTick, Timer32_getValue(ULTRA_TIMER_BASE));
        if (elapsedTicks > (ROBOT_CFG_ULTRA_ECHO_TIMEOUT_US * ticksPerUs)) {
            return ROBOT_CFG_ULTRA_INVALID_CM;
        }
    }
    pulseEndTick = Timer32_getValue(ULTRA_TIMER_BASE);

    elapsedTicks = Ultrasonic_TimerDelta(pulseStartTick, pulseEndTick);
    pulseUs = (float)elapsedTicks / (float)ticksPerUs;

    /*
     * HC-SR04 style conversion:
     * distance_cm = echo_pulse_us / 58
     *
     * Example higher-level logic:
     * - if absolute difference between left and right is small, floor is normal
     * - if left differs strongly from baseline, something unusual is on the left
     * - if right differs strongly from baseline, something unusual is on the right
     *
     * In practice, store a floor baseline after mounting and compare against a
     * tolerance instead of exact equality.
     */
    return pulseUs / 58.0f;
}

static uint32_t Ultrasonic_TimerDelta(uint32_t startTick, uint32_t endTick)
{
    if (startTick >= endTick) {
        return startTick - endTick;
    }

    return startTick + ((ULTRA_TIMER_MAX_COUNT - endTick) + 1U);
}
