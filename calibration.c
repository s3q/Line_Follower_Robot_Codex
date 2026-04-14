#include "calibration.h"
#include "bump.h"
#include "line.h"
#include "robot_config.h"
#include "robot_pins.h"
#include "ultrasonic.h"
#include <stdbool.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

volatile CalibrationData gCalibrationData = {0};

static void Calibration_DelayMs(uint32_t ms);
static void Calibration_SetLeds(bool redOn, bool greenOn, bool blueOn);
static void Calibration_ClearLineArrays(void);
static void Calibration_CaptureLineCounts(uint16_t *counts, uint32_t sampleCount);
static void Calibration_ComputeThresholds(uint32_t sampleCount);
static float Calibration_AverageUltrasonic(bool leftSensor, uint32_t sampleCount);

void Calibration_Init(void)
{
    GPIO_setAsOutputPin(LED_RED_PORT, LED_RED_PIN);
    GPIO_setAsOutputPin(LED_GREEN_PORT, LED_GREEN_PIN);
    GPIO_setAsOutputPin(LED_BLUE_PORT, LED_BLUE_PIN);

    Calibration_SetLeds(false, false, false);

    Bump_Init();
    Line_Init();
    Ultrasonic_Init();
}

void Calibration_Run(void)
{
    uint32_t i;

    /*
     * Calibration sequence:
     * 1) Red LED stage: verify bump switches respond.
     * 2) Green LED stage: place robot over white surface for line sampling.
     * 3) Blue LED stage: place robot over black tape/line for line sampling.
     * 4) Green+Blue stage: place robot on normal flat floor for ultrasonic baseline.
     * 5) All LEDs on: calibration complete.
     *
     * Line calibration should be done with the robot placed over white first,
     * then over black. Ultrasonic calibration should be done on normal flat
     * floor at the real mounting angle. These constants will later be used by
     * line following, obstacle detection, and collision logic.
     */

    Calibration_ClearLineArrays();
    gCalibrationData.ultrasonicLeftBaselineCm = -1.0f;
    gCalibrationData.ultrasonicRightBaselineCm = -1.0f;

    /* Stage 1: bump verification support. */
    Calibration_SetLeds(true, false, false);
    for (i = 0U; i < ROBOT_CFG_CAL_BUMP_VERIFY_MS; i++) {
        gCalibrationData.latestBumpState = Bump_Read();
        if (gCalibrationData.latestBumpState != 0U) {
            GPIO_toggleOutputOnPin(LED_RED_PORT, LED_RED_PIN);
        }
        Calibration_DelayMs(1U);
    }
    Calibration_SetLeds(false, false, false);

    /* Stage 2: white surface line calibration. */
    Calibration_SetLeds(false, true, false);
    Calibration_DelayMs(ROBOT_CFG_CAL_STAGE_PREP_MS);
    Calibration_CaptureLineCounts((uint16_t *)gCalibrationData.lineWhiteCounts, ROBOT_CFG_CAL_LINE_SAMPLE_COUNT);
    Calibration_SetLeds(false, false, false);

    /* Stage 3: black line/tape calibration. */
    Calibration_SetLeds(false, false, true);
    Calibration_DelayMs(ROBOT_CFG_CAL_STAGE_PREP_MS);
    Calibration_CaptureLineCounts((uint16_t *)gCalibrationData.lineBlackCounts, ROBOT_CFG_CAL_LINE_SAMPLE_COUNT);
    Calibration_SetLeds(false, false, false);

    Calibration_ComputeThresholds(ROBOT_CFG_CAL_LINE_SAMPLE_COUNT);

    /* Stage 4: ultrasonic floor baseline. */
    Calibration_SetLeds(false, true, true);
    Calibration_DelayMs(ROBOT_CFG_CAL_STAGE_PREP_MS);
    gCalibrationData.ultrasonicLeftBaselineCm = Calibration_AverageUltrasonic(true, ROBOT_CFG_CAL_ULTRA_SAMPLE_COUNT);
    gCalibrationData.ultrasonicRightBaselineCm = Calibration_AverageUltrasonic(false, ROBOT_CFG_CAL_ULTRA_SAMPLE_COUNT);

    /* Stage 5: done indication. */
    Calibration_SetLeds(true, true, true);
    Calibration_DelayMs(ROBOT_CFG_CAL_STAGE_DONE_MS);
}

static void Calibration_DelayMs(uint32_t ms)
{
    uint32_t mclkHz;
    uint32_t ticksPerMs;
    uint32_t ticksNeeded;
    uint32_t elapsed;
    uint32_t loadValue;
    uint32_t lastTick;
    uint32_t currentTick;

    mclkHz = CS_getMCLK();
    if (mclkHz == 0U) {
        mclkHz = 3000000U;
    }

    ticksPerMs = mclkHz / 1000U;
    ticksNeeded = ms * ticksPerMs;
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

static void Calibration_SetLeds(bool redOn, bool greenOn, bool blueOn)
{
    if (redOn) {
        GPIO_setOutputHighOnPin(LED_RED_PORT, LED_RED_PIN);
    } else {
        GPIO_setOutputLowOnPin(LED_RED_PORT, LED_RED_PIN);
    }

    if (greenOn) {
        GPIO_setOutputHighOnPin(LED_GREEN_PORT, LED_GREEN_PIN);
    } else {
        GPIO_setOutputLowOnPin(LED_GREEN_PORT, LED_GREEN_PIN);
    }

    if (blueOn) {
        GPIO_setOutputHighOnPin(LED_BLUE_PORT, LED_BLUE_PIN);
    } else {
        GPIO_setOutputLowOnPin(LED_BLUE_PORT, LED_BLUE_PIN);
    }
}

static void Calibration_ClearLineArrays(void)
{
    uint32_t i;

    for (i = 0U; i < 8U; i++) {
        gCalibrationData.lineWhiteCounts[i] = 0U;
        gCalibrationData.lineBlackCounts[i] = 0U;
        gCalibrationData.lineThresholdCounts[i] = 0U;
    }
}

static void Calibration_CaptureLineCounts(uint16_t *counts, uint32_t sampleCount)
{
    uint32_t sample;
    uint32_t bit;
    uint8_t lineReading;

    for (sample = 0U; sample < sampleCount; sample++) {
        lineReading = Line_Read(ROBOT_CFG_CAL_LINE_SAMPLE_US);
        gCalibrationData.latestLineReading = lineReading;
        gCalibrationData.latestBumpState = Bump_Read();

        for (bit = 0U; bit < 8U; bit++) {
            if ((lineReading & (1U << bit)) != 0U) {
                counts[bit]++;
            }
        }

        Calibration_DelayMs(20U);
    }
}

static void Calibration_ComputeThresholds(uint32_t sampleCount)
{
    uint32_t i;

    for (i = 0U; i < 8U; i++) {
        /*
         * Threshold is stored in the same "count over N samples" space as the
         * white/black captures. Later code can compare a filtered live count
         * against this midpoint threshold.
         */
        (void)sampleCount;
        gCalibrationData.lineThresholdCounts[i] =
            (uint16_t)((gCalibrationData.lineWhiteCounts[i] +
                        gCalibrationData.lineBlackCounts[i]) / 2U);
    }
}

static float Calibration_AverageUltrasonic(bool leftSensor, uint32_t sampleCount)
{
    uint32_t i;
    uint32_t validCount;
    float sum;
    float reading;

    sum = 0.0f;
    validCount = 0U;

    for (i = 0U; i < sampleCount; i++) {
        if (leftSensor) {
            reading = Ultrasonic_ReadLeftCm();
        } else {
            reading = Ultrasonic_ReadRightCm();
        }

        if (reading > 0.0f) {
            sum += reading;
            validCount++;
        }

        Calibration_DelayMs(ROBOT_CFG_CAL_ULTRA_INTER_SAMPLE_MS);
    }

    if (validCount == 0U) {
        return -1.0f;
    }

    return sum / (float)validCount;
}
