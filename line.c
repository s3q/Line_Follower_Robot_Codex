#include "line.h"
#include "robot_pins.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

static void Line_DelayUs(uint32_t us);

void Line_Init(void)
{
    /* IR emitters off at startup. */
    GPIO_setAsOutputPin(LINE_EMIT_LEFT_PORT, LINE_EMIT_LEFT_PIN);
    GPIO_setAsOutputPin(LINE_EMIT_RIGHT_PORT, LINE_EMIT_RIGHT_PIN);
    GPIO_setOutputLowOnPin(LINE_EMIT_LEFT_PORT, LINE_EMIT_LEFT_PIN);
    GPIO_setOutputLowOnPin(LINE_EMIT_RIGHT_PORT, LINE_EMIT_RIGHT_PIN);

    /* Sensor array pins P7.7..P7.0 as GPIO inputs initially. */
    GPIO_setAsInputPin(LINE_SENSOR_PORT, LINE_SENSOR_ALL_PINS);
}

uint8_t Line_Read(uint32_t sample_us)
{
    uint8_t lineBits;

    /* 1) Emitters ON */
    GPIO_setOutputHighOnPin(LINE_EMIT_LEFT_PORT, LINE_EMIT_LEFT_PIN);
    GPIO_setOutputHighOnPin(LINE_EMIT_RIGHT_PORT, LINE_EMIT_RIGHT_PIN);

    /* 2) Drive P7.7..P7.0 high to charge sensor capacitors */
    GPIO_setAsOutputPin(LINE_SENSOR_PORT, LINE_SENSOR_ALL_PINS);
    GPIO_setOutputHighOnPin(LINE_SENSOR_PORT, LINE_SENSOR_ALL_PINS);

    /* 3) Wait about 10 us */
    Line_DelayUs(10U);

    /* 4) Change P7.7..P7.0 to inputs */
    GPIO_setAsInputPin(LINE_SENSOR_PORT, LINE_SENSOR_ALL_PINS);

    /* 5) Wait configurable sample time */
    Line_DelayUs(sample_us);

    /*
     * 6) Read P7.7..P7.0 as 8-bit value.
     * Bit mapping is direct: bit0=P7.0 ... bit7=P7.7
     * Convention used here: 0=white, 1=black.
     */
    lineBits = 0U;
    lineBits |= ((GPIO_getInputPinValue(LINE_SENSOR_PORT, LINE_SENSOR_BIT0_PIN) != 0U) ? 1U : 0U) << 0;
    lineBits |= ((GPIO_getInputPinValue(LINE_SENSOR_PORT, LINE_SENSOR_BIT1_PIN) != 0U) ? 1U : 0U) << 1;
    lineBits |= ((GPIO_getInputPinValue(LINE_SENSOR_PORT, LINE_SENSOR_BIT2_PIN) != 0U) ? 1U : 0U) << 2;
    lineBits |= ((GPIO_getInputPinValue(LINE_SENSOR_PORT, LINE_SENSOR_BIT3_PIN) != 0U) ? 1U : 0U) << 3;
    lineBits |= ((GPIO_getInputPinValue(LINE_SENSOR_PORT, LINE_SENSOR_BIT4_PIN) != 0U) ? 1U : 0U) << 4;
    lineBits |= ((GPIO_getInputPinValue(LINE_SENSOR_PORT, LINE_SENSOR_BIT5_PIN) != 0U) ? 1U : 0U) << 5;
    lineBits |= ((GPIO_getInputPinValue(LINE_SENSOR_PORT, LINE_SENSOR_BIT6_PIN) != 0U) ? 1U : 0U) << 6;
    lineBits |= ((GPIO_getInputPinValue(LINE_SENSOR_PORT, LINE_SENSOR_BIT7_PIN) != 0U) ? 1U : 0U) << 7;

    /* 7) Emitters OFF */
    GPIO_setOutputLowOnPin(LINE_EMIT_LEFT_PORT, LINE_EMIT_LEFT_PIN);
    GPIO_setOutputLowOnPin(LINE_EMIT_RIGHT_PORT, LINE_EMIT_RIGHT_PIN);

    /* 8) Return reading */
    return lineBits;
}

static void Line_DelayUs(uint32_t us)
{
    uint32_t mclkHz = CS_getMCLK();
    uint32_t ticksPerUs = mclkHz / 1000000U;
    uint32_t ticksNeeded;
    uint32_t elapsed;
    uint32_t loadValue;
    uint32_t lastTick;
    uint32_t currentTick;

    if (ticksPerUs == 0U) {
        ticksPerUs = 3U; /* default MSP432 reset clock is ~3 MHz */
    }

    /*
     * Use the active SysTick hardware counter instead of __delay_cycles().
     * This matches the project's working timing strategy and avoids
     * reintroducing cycle-count timing drift inside Line_Read().
     */
    ticksNeeded = us * ticksPerUs;
    elapsed = 0U;
    loadValue = SysTick->LOAD + 1U;
    lastTick = SysTick->VAL;

    while (elapsed < ticksNeeded) {
        currentTick = SysTick->VAL;

        if (currentTick <= lastTick) {
            elapsed += lastTick - currentTick;
        } else {
            elapsed += lastTick + (loadValue - currentTick);
        }

        lastTick = currentTick;
    }
}
