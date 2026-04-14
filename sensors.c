#include "sensors.h"
#include "bump.h"
#include "ir.h"
#include "line.h"
#include "robot_config.h"
#include "robot_pins.h"
#include <stdbool.h>
#include <string.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#define SENSORS_TIMER_BASE                  TIMER32_1_BASE
#define SENSORS_TIMER_MAX_COUNT             0xFFFFFFFFU
#define SENSOR_SIDE_LEFT                    0U
#define SENSOR_SIDE_RIGHT                   1U

typedef enum {
    LINE_SM_IDLE = 0,
    LINE_SM_CHARGE_WAIT = 1,
    LINE_SM_SAMPLE_WAIT = 2
} LineSmState;

typedef enum {
    US_SM_IDLE = 0,
    US_SM_TRIG_HIGH = 1,
    US_SM_WAIT_RISE = 2,
    US_SM_WAIT_FALL = 3
} UsSmState;

typedef enum {
    IR_SM_IDLE = 0,
    IR_SM_WAIT = 1
} IrSmState;

typedef enum {
    IMU_SM_IDLE = 0,
    IMU_SM_WAIT = 1
} ImuSmState;

volatile SensorSnapshot gSensorSnapshot;
volatile SensorsDebug gSensorsDebug;

static bool gSensorsInitDone = false;
static uint32_t gTicksPerUs = 3U;
static uint32_t gLastUpdateMs = 0U;

static LineSmState gLineSmState = LINE_SM_IDLE;
static uint32_t gLineSmStartTick = 0U;
static uint32_t gLineNextDueMs = 0U;

static UsSmState gUsSmState = US_SM_IDLE;
static uint8_t gUsActiveSide = SENSOR_SIDE_LEFT;
static uint32_t gUsSmStartTick = 0U;
static uint32_t gUsEchoRiseTick = 0U;
static uint32_t gUsNextDueMs[2] = {0U, SENSORS_ULTRA_PERIOD_MS};

static IrSmState gIrSmState = IR_SM_IDLE;
static uint32_t gIrSmStartTick = 0U;
static uint32_t gIrNextDueMs = 0U;

static ImuSmState gImuSmState = IMU_SM_IDLE;
#if SENSORS_ENABLE_IMU_HOOK
static uint32_t gImuSmStartTick = 0U;
#endif
static uint32_t gImuNextDueMs = 0U;

static uint32_t gBumpNextDueMs = 0U;
static uint8_t gBumpLastRaw = 0U;
static uint8_t gBumpDebounced = 0U;
static uint8_t gBumpStableCount = 0U;

static uint8_t gStaleLine = 1U;
static uint8_t gStaleUsLeft = 1U;
static uint8_t gStaleUsRight = 1U;
static uint8_t gStaleIr = 1U;
static uint8_t gStaleImu = 1U;
static uint8_t gStaleBump = 1U;

bool SensorsImuHook_StartRead(void) __attribute__((weak));
bool SensorsImuHook_PollRead(SensorImuSample *out) __attribute__((weak));
void SensorsImuHook_Init(void) __attribute__((weak));
void SensorsImuHook_AbortRead(void) __attribute__((weak));

static uint32_t Sensors_TimerDeltaTicks(uint32_t startTick, uint32_t endTick);
static uint32_t Sensors_TimerDeltaUs(uint32_t startTick, uint32_t endTick);
static uint8_t Sensors_PopCount8(uint8_t value);
static int16_t Sensors_ComputeLineError(uint8_t mask, uint8_t count);
static uint8_t Sensors_ReadLineMaskNow(void);
static void Sensors_LineSetEmittersOn(void);
static void Sensors_LineSetEmittersOff(void);
static void Sensors_LineAbort(uint32_t nowMs);
static void Sensors_ServiceLine(uint32_t nowMs, uint32_t nowTick);
static void Sensors_UltraAbort(uint32_t nowMs, uint32_t errMask);
static void Sensors_ServiceUltrasonic(uint32_t nowMs, uint32_t nowTick);
static uint16_t Sensors_IrRawToMm(uint16_t raw);
static void Sensors_ServiceIr(uint32_t nowMs, uint32_t nowTick);
static void Sensors_ServiceBump(uint32_t nowMs);
static void Sensors_ServiceImu(uint32_t nowMs, uint32_t nowTick);
static uint16_t Sensors_AgeMsFromDiag(const SensorDiag *diag, uint32_t nowMs);
static void Sensors_UpdateStaleCounters(uint32_t nowMs);
static void Sensors_DiagKick(volatile SensorDiag *diag);
static void Sensors_DiagOk(volatile SensorDiag *diag, uint32_t nowMs);
static void Sensors_DiagTimeout(volatile SensorDiag *diag, uint32_t nowMs);
#if SENSORS_ENABLE_IMU_HOOK
static void Sensors_DiagHwError(volatile SensorDiag *diag, uint32_t nowMs);
#endif
static void Sensors_DiagTrackStale(volatile SensorDiag *diag, uint8_t stale, uint8_t *prevStale);
static uint_fast8_t Sensors_UltraTrigPort(uint8_t side);
static uint_fast16_t Sensors_UltraTrigPin(uint8_t side);
static uint_fast8_t Sensors_UltraEchoPort(uint8_t side);
static uint_fast16_t Sensors_UltraEchoPin(uint8_t side);
static uint32_t Sensors_UltraErrMask(uint8_t side);
static volatile SensorDiag *Sensors_UltraDiag(uint8_t side);

bool SensorsImuHook_StartRead(void)
{
    return false;
}

bool SensorsImuHook_PollRead(SensorImuSample *out)
{
    (void)out;
    return false;
}

void SensorsImuHook_Init(void)
{
}

void SensorsImuHook_AbortRead(void)
{
}

void Sensors_Init(void)
{
    uint32_t mclkHz;

    memset((void *)&gSensorSnapshot, 0, sizeof(gSensorSnapshot));
    memset((void *)&gSensorsDebug, 0, sizeof(gSensorsDebug));

    mclkHz = CS_getMCLK();
    gTicksPerUs = mclkHz / 1000000U;
    if (gTicksPerUs == 0U) {
        gTicksPerUs = 3U;
    }

    Timer32_initModule(SENSORS_TIMER_BASE,
                       TIMER32_PRESCALER_1,
                       TIMER32_32BIT,
                       TIMER32_PERIODIC_MODE);
    Timer32_setCount(SENSORS_TIMER_BASE, SENSORS_TIMER_MAX_COUNT);
    Timer32_startTimer(SENSORS_TIMER_BASE, false);

    Line_Init();
    IR_Init();
    Bump_Init();

    GPIO_setAsOutputPin(ULTRA_LEFT_TRIG_PORT, ULTRA_LEFT_TRIG_PIN);
    GPIO_setAsOutputPin(ULTRA_RIGHT_TRIG_PORT, ULTRA_RIGHT_TRIG_PIN);
    GPIO_setAsInputPin(ULTRA_LEFT_ECHO_PORT, ULTRA_LEFT_ECHO_PIN);
    GPIO_setAsInputPin(ULTRA_RIGHT_ECHO_PORT, ULTRA_RIGHT_ECHO_PIN);
    GPIO_setOutputLowOnPin(ULTRA_LEFT_TRIG_PORT, ULTRA_LEFT_TRIG_PIN);
    GPIO_setOutputLowOnPin(ULTRA_RIGHT_TRIG_PORT, ULTRA_RIGHT_TRIG_PIN);

#if SENSORS_ENABLE_IMU_HOOK
    SensorsImuHook_Init();
#endif

    gLineSmState = LINE_SM_IDLE;
    gUsSmState = US_SM_IDLE;
    gIrSmState = IR_SM_IDLE;
    gImuSmState = IMU_SM_IDLE;
    gLineNextDueMs = 0U;
    gUsNextDueMs[SENSOR_SIDE_LEFT] = 0U;
    gUsNextDueMs[SENSOR_SIDE_RIGHT] = SENSORS_ULTRA_PERIOD_MS;
    gIrNextDueMs = 0U;
    gImuNextDueMs = 0U;
    gBumpNextDueMs = 0U;
    gBumpLastRaw = 0U;
    gBumpDebounced = 0U;
    gBumpStableCount = 0U;
    gStaleLine = 1U;
    gStaleUsLeft = 1U;
    gStaleUsRight = 1U;
    gStaleIr = 1U;
    gStaleImu = 1U;
    gStaleBump = 1U;
    gLastUpdateMs = 0U;
    gSensorsInitDone = true;
}

void Sensors_Update(uint32_t now_ms)
{
    uint32_t nowTick;
    uint32_t dtMs;

    if (!gSensorsInitDone) {
        Sensors_Init();
    }

    if (gLastUpdateMs != 0U) {
        dtMs = now_ms - gLastUpdateMs;
        if (dtMs > (SENSORS_CONTROL_PERIOD_MS + 1U)) {
            gSensorsDebug.loop_missed_deadline_count++;
        }
        if (dtMs > (SENSORS_CONTROL_PERIOD_MS * 2U)) {
            gSensorsDebug.loop_overrun_count++;
        }
    }
    gLastUpdateMs = now_ms;

    nowTick = Timer32_getValue(SENSORS_TIMER_BASE);

    Sensors_ServiceLine(now_ms, nowTick);
    Sensors_ServiceUltrasonic(now_ms, nowTick);
    Sensors_ServiceIr(now_ms, nowTick);
    Sensors_ServiceImu(now_ms, nowTick);
    Sensors_ServiceBump(now_ms);

    Sensors_UpdateStaleCounters(now_ms);

    gSensorSnapshot.ts_ms = now_ms;
    gSensorSnapshot.seq++;
}

void Sensors_ClearErrors(void)
{
    gSensorSnapshot.error_flags = 0U;
}

const volatile SensorSnapshot *Sensors_GetSnapshot(void)
{
    return &gSensorSnapshot;
}

const volatile SensorsDebug *Sensors_GetDebug(void)
{
    return &gSensorsDebug;
}

static uint32_t Sensors_TimerDeltaTicks(uint32_t startTick, uint32_t endTick)
{
    if (startTick >= endTick) {
        return startTick - endTick;
    }

    return startTick + ((SENSORS_TIMER_MAX_COUNT - endTick) + 1U);
}

static uint32_t Sensors_TimerDeltaUs(uint32_t startTick, uint32_t endTick)
{
    uint32_t deltaTicks = Sensors_TimerDeltaTicks(startTick, endTick);
    return deltaTicks / gTicksPerUs;
}

static uint8_t Sensors_PopCount8(uint8_t value)
{
    uint8_t count = 0U;
    while (value != 0U) {
        count += (uint8_t)(value & 1U);
        value >>= 1;
    }
    return count;
}

static int16_t Sensors_ComputeLineError(uint8_t mask, uint8_t count)
{
    int32_t weightedSum = 0;
    uint8_t bit;

    if (count == 0U) {
        return 0;
    }

    for (bit = 0U; bit < 8U; bit++) {
        if ((mask & (uint8_t)(1U << bit)) != 0U) {
            weightedSum += (int32_t)bit * 1000;
        }
    }

    return (int16_t)(3500 - (weightedSum / (int32_t)count));
}

static uint8_t Sensors_ReadLineMaskNow(void)
{
    uint8_t lineBits = 0U;

    lineBits |= ((GPIO_getInputPinValue(LINE_SENSOR_PORT, LINE_SENSOR_BIT0_PIN) != 0U) ? 1U : 0U) << 0;
    lineBits |= ((GPIO_getInputPinValue(LINE_SENSOR_PORT, LINE_SENSOR_BIT1_PIN) != 0U) ? 1U : 0U) << 1;
    lineBits |= ((GPIO_getInputPinValue(LINE_SENSOR_PORT, LINE_SENSOR_BIT2_PIN) != 0U) ? 1U : 0U) << 2;
    lineBits |= ((GPIO_getInputPinValue(LINE_SENSOR_PORT, LINE_SENSOR_BIT3_PIN) != 0U) ? 1U : 0U) << 3;
    lineBits |= ((GPIO_getInputPinValue(LINE_SENSOR_PORT, LINE_SENSOR_BIT4_PIN) != 0U) ? 1U : 0U) << 4;
    lineBits |= ((GPIO_getInputPinValue(LINE_SENSOR_PORT, LINE_SENSOR_BIT5_PIN) != 0U) ? 1U : 0U) << 5;
    lineBits |= ((GPIO_getInputPinValue(LINE_SENSOR_PORT, LINE_SENSOR_BIT6_PIN) != 0U) ? 1U : 0U) << 6;
    lineBits |= ((GPIO_getInputPinValue(LINE_SENSOR_PORT, LINE_SENSOR_BIT7_PIN) != 0U) ? 1U : 0U) << 7;

    return lineBits;
}

static void Sensors_LineSetEmittersOn(void)
{
    GPIO_setOutputHighOnPin(LINE_EMIT_LEFT_PORT, LINE_EMIT_LEFT_PIN);
    GPIO_setOutputHighOnPin(LINE_EMIT_RIGHT_PORT, LINE_EMIT_RIGHT_PIN);
}

static void Sensors_LineSetEmittersOff(void)
{
    GPIO_setOutputLowOnPin(LINE_EMIT_LEFT_PORT, LINE_EMIT_LEFT_PIN);
    GPIO_setOutputLowOnPin(LINE_EMIT_RIGHT_PORT, LINE_EMIT_RIGHT_PIN);
}

static void Sensors_LineAbort(uint32_t nowMs)
{
    Sensors_LineSetEmittersOff();
    GPIO_setAsInputPin(LINE_SENSOR_PORT, LINE_SENSOR_ALL_PINS);

    gSensorSnapshot.error_flags |= SENSOR_ERR_LINE_TIMEOUT;
    gSensorSnapshot.line.valid = 0U;
    Sensors_DiagTimeout(&gSensorsDebug.line, nowMs);

    gLineSmState = LINE_SM_IDLE;
    gLineNextDueMs = nowMs + SENSORS_LINE_PERIOD_MS;
}

static void Sensors_ServiceLine(uint32_t nowMs, uint32_t nowTick)
{
    uint32_t elapsedUs;

    switch (gLineSmState) {
    case LINE_SM_IDLE:
        if (nowMs < gLineNextDueMs) {
            return;
        }

        Sensors_DiagKick(&gSensorsDebug.line);
        Sensors_LineSetEmittersOn();
        GPIO_setAsOutputPin(LINE_SENSOR_PORT, LINE_SENSOR_ALL_PINS);
        GPIO_setOutputHighOnPin(LINE_SENSOR_PORT, LINE_SENSOR_ALL_PINS);
        gLineSmStartTick = nowTick;
        gLineSmState = LINE_SM_CHARGE_WAIT;
        return;

    case LINE_SM_CHARGE_WAIT:
        elapsedUs = Sensors_TimerDeltaUs(gLineSmStartTick, nowTick);
        if (elapsedUs >= 10U) {
            GPIO_setAsInputPin(LINE_SENSOR_PORT, LINE_SENSOR_ALL_PINS);
            gLineSmStartTick = nowTick;
            gLineSmState = LINE_SM_SAMPLE_WAIT;
            return;
        }
        if (elapsedUs > SENSORS_LINE_TIMEOUT_US) {
            Sensors_LineAbort(nowMs);
        }
        return;

    case LINE_SM_SAMPLE_WAIT:
        elapsedUs = Sensors_TimerDeltaUs(gLineSmStartTick, nowTick);
        if (elapsedUs >= SENSORS_LINE_SAMPLE_US) {
            uint8_t mask = Sensors_ReadLineMaskNow();
            uint8_t count = Sensors_PopCount8(mask);

            Sensors_LineSetEmittersOff();
            gSensorSnapshot.line.mask = mask;
            gSensorSnapshot.line.count = count;
            gSensorSnapshot.line.line_err = Sensors_ComputeLineError(mask, count);
            gSensorSnapshot.line.valid = 1U;
            gSensorSnapshot.error_flags &= ~SENSOR_ERR_LINE_TIMEOUT;
            Sensors_DiagOk(&gSensorsDebug.line, nowMs);

            gLineSmState = LINE_SM_IDLE;
            gLineNextDueMs = nowMs + SENSORS_LINE_PERIOD_MS;
            return;
        }
        if (elapsedUs > SENSORS_LINE_TIMEOUT_US) {
            Sensors_LineAbort(nowMs);
        }
        return;

    default:
        gLineSmState = LINE_SM_IDLE;
        return;
    }
}

static uint_fast8_t Sensors_UltraTrigPort(uint8_t side)
{
    return (side == SENSOR_SIDE_LEFT) ? ULTRA_LEFT_TRIG_PORT : ULTRA_RIGHT_TRIG_PORT;
}

static uint_fast16_t Sensors_UltraTrigPin(uint8_t side)
{
    return (side == SENSOR_SIDE_LEFT) ? ULTRA_LEFT_TRIG_PIN : ULTRA_RIGHT_TRIG_PIN;
}

static uint_fast8_t Sensors_UltraEchoPort(uint8_t side)
{
    return (side == SENSOR_SIDE_LEFT) ? ULTRA_LEFT_ECHO_PORT : ULTRA_RIGHT_ECHO_PORT;
}

static uint_fast16_t Sensors_UltraEchoPin(uint8_t side)
{
    return (side == SENSOR_SIDE_LEFT) ? ULTRA_LEFT_ECHO_PIN : ULTRA_RIGHT_ECHO_PIN;
}

static uint32_t Sensors_UltraErrMask(uint8_t side)
{
    return (side == SENSOR_SIDE_LEFT) ? SENSOR_ERR_ULTRA_LEFT_TIMEOUT : SENSOR_ERR_ULTRA_RIGHT_TIMEOUT;
}

static volatile SensorDiag *Sensors_UltraDiag(uint8_t side)
{
    return (side == SENSOR_SIDE_LEFT) ? &gSensorsDebug.us_left : &gSensorsDebug.us_right;
}

static void Sensors_UltraAbort(uint32_t nowMs, uint32_t errMask)
{
    uint8_t side = gUsActiveSide;

    GPIO_setOutputLowOnPin(Sensors_UltraTrigPort(side), Sensors_UltraTrigPin(side));

    gSensorSnapshot.error_flags |= errMask;
    if (side == SENSOR_SIDE_LEFT) {
        gSensorSnapshot.us.left_valid = 0U;
    } else {
        gSensorSnapshot.us.right_valid = 0U;
    }
    Sensors_DiagTimeout(Sensors_UltraDiag(side), nowMs);

    gUsNextDueMs[side] = nowMs + (2U * SENSORS_ULTRA_PERIOD_MS);
    gUsSmState = US_SM_IDLE;
}

static void Sensors_ServiceUltrasonic(uint32_t nowMs, uint32_t nowTick)
{
    uint8_t side;
    uint32_t elapsedUs;

    switch (gUsSmState) {
    case US_SM_IDLE:
        if (nowMs >= gUsNextDueMs[SENSOR_SIDE_LEFT]) {
            gUsActiveSide = SENSOR_SIDE_LEFT;
        } else if (nowMs >= gUsNextDueMs[SENSOR_SIDE_RIGHT]) {
            gUsActiveSide = SENSOR_SIDE_RIGHT;
        } else {
            return;
        }

        side = gUsActiveSide;
        Sensors_DiagKick(Sensors_UltraDiag(side));
        GPIO_setOutputLowOnPin(Sensors_UltraTrigPort(side), Sensors_UltraTrigPin(side));
        GPIO_setOutputHighOnPin(Sensors_UltraTrigPort(side), Sensors_UltraTrigPin(side));
        gUsSmStartTick = nowTick;
        gUsSmState = US_SM_TRIG_HIGH;
        return;

    case US_SM_TRIG_HIGH:
        elapsedUs = Sensors_TimerDeltaUs(gUsSmStartTick, nowTick);
        if (elapsedUs >= ROBOT_CFG_ULTRA_TRIGGER_PULSE_US) {
            side = gUsActiveSide;
            GPIO_setOutputLowOnPin(Sensors_UltraTrigPort(side), Sensors_UltraTrigPin(side));
            gUsSmStartTick = nowTick;
            gUsSmState = US_SM_WAIT_RISE;
            return;
        }
        return;

    case US_SM_WAIT_RISE:
        side = gUsActiveSide;
        if (GPIO_getInputPinValue(Sensors_UltraEchoPort(side), Sensors_UltraEchoPin(side)) != 0U) {
            gUsEchoRiseTick = nowTick;
            gUsSmStartTick = nowTick;
            gUsSmState = US_SM_WAIT_FALL;
            return;
        }

        elapsedUs = Sensors_TimerDeltaUs(gUsSmStartTick, nowTick);
        if (elapsedUs > SENSORS_US_START_TIMEOUT_US) {
            Sensors_UltraAbort(nowMs, Sensors_UltraErrMask(side));
        }
        return;

    case US_SM_WAIT_FALL:
        side = gUsActiveSide;
        if (GPIO_getInputPinValue(Sensors_UltraEchoPort(side), Sensors_UltraEchoPin(side)) == 0U) {
            uint32_t pulseUs = Sensors_TimerDeltaUs(gUsEchoRiseTick, nowTick);
            uint32_t mm = (pulseUs * 10U + 29U) / 58U;
            if (mm > 0xFFFFU) {
                mm = 0xFFFFU;
            }

            if (side == SENSOR_SIDE_LEFT) {
                gSensorSnapshot.us.left_mm = (uint16_t)mm;
                gSensorSnapshot.us.left_valid = 1U;
            } else {
                gSensorSnapshot.us.right_mm = (uint16_t)mm;
                gSensorSnapshot.us.right_valid = 1U;
            }
            gSensorSnapshot.error_flags &= ~Sensors_UltraErrMask(side);
            Sensors_DiagOk(Sensors_UltraDiag(side), nowMs);

            gUsNextDueMs[side] = nowMs + (2U * SENSORS_ULTRA_PERIOD_MS);
            gUsSmState = US_SM_IDLE;
            return;
        }

        elapsedUs = Sensors_TimerDeltaUs(gUsSmStartTick, nowTick);
        if (elapsedUs > SENSORS_US_ECHO_TIMEOUT_US) {
            Sensors_UltraAbort(nowMs, Sensors_UltraErrMask(side));
        }
        return;

    default:
        gUsSmState = US_SM_IDLE;
        return;
    }
}

static uint16_t Sensors_IrRawToMm(uint16_t raw)
{
    if (raw < 80U) {
        return 0xFFFFU;
    }

    /* Simple inverse fit for watch/debug; replace with calibrated table if needed. */
    {
        uint32_t mm = 700000U / raw;
        if (mm > 0xFFFFU) {
            mm = 0xFFFFU;
        }
        return (uint16_t)mm;
    }
}

static void Sensors_ServiceIr(uint32_t nowMs, uint32_t nowTick)
{
    uint32_t elapsedUs;

    switch (gIrSmState) {
    case IR_SM_IDLE:
        if (nowMs < gIrNextDueMs) {
            return;
        }

        Sensors_DiagKick(&gSensorsDebug.ir);
        ADC14_toggleConversionTrigger();
        gIrSmStartTick = nowTick;
        gIrSmState = IR_SM_WAIT;
        return;

    case IR_SM_WAIT:
        if (!ADC14_isBusy()) {
            uint16_t raw = (uint16_t)ADC14_getResult(ADC_MEM0);
            gSensorSnapshot.ir.raw = raw;
            gSensorSnapshot.ir.dist_mm = Sensors_IrRawToMm(raw);
            gSensorSnapshot.ir.valid = 1U;
            gSensorSnapshot.error_flags &= ~SENSOR_ERR_IR_TIMEOUT;
            Sensors_DiagOk(&gSensorsDebug.ir, nowMs);

            gIrSmState = IR_SM_IDLE;
            gIrNextDueMs = nowMs + SENSORS_IR_PERIOD_MS;
            return;
        }

        elapsedUs = Sensors_TimerDeltaUs(gIrSmStartTick, nowTick);
        if (elapsedUs > SENSORS_IR_TIMEOUT_US) {
            gSensorSnapshot.error_flags |= SENSOR_ERR_IR_TIMEOUT;
            gSensorSnapshot.ir.valid = 0U;
            Sensors_DiagTimeout(&gSensorsDebug.ir, nowMs);

            gIrSmState = IR_SM_IDLE;
            gIrNextDueMs = nowMs + SENSORS_IR_PERIOD_MS;
        }
        return;

    default:
        gIrSmState = IR_SM_IDLE;
        return;
    }
}

static void Sensors_ServiceBump(uint32_t nowMs)
{
    uint8_t raw;

    if (nowMs < gBumpNextDueMs) {
        return;
    }

    Sensors_DiagKick(&gSensorsDebug.bump);
    raw = Bump_Read();
    gSensorSnapshot.bump.raw_mask = raw;

    if (raw == gBumpLastRaw) {
        if (gBumpStableCount < 0xFFU) {
            gBumpStableCount++;
        }
    } else {
        gBumpLastRaw = raw;
        gBumpStableCount = 0U;
    }

    if (gBumpStableCount >= SENSORS_BUMP_DEBOUNCE_SAMPLES) {
        gBumpDebounced = raw;
    }

    gSensorSnapshot.bump.debounced_mask = gBumpDebounced;
    gSensorSnapshot.bump.valid = 1U;
    Sensors_DiagOk(&gSensorsDebug.bump, nowMs);
    gBumpNextDueMs = nowMs + SENSORS_BUMP_PERIOD_MS;
}

static void Sensors_ServiceImu(uint32_t nowMs, uint32_t nowTick)
{
#if SENSORS_ENABLE_IMU_HOOK
    uint32_t elapsedUs;
    SensorImuSample sample;

    switch (gImuSmState) {
    case IMU_SM_IDLE:
        if (nowMs < gImuNextDueMs) {
            return;
        }

        Sensors_DiagKick(&gSensorsDebug.imu);
        if (SensorsImuHook_StartRead()) {
            gImuSmStartTick = nowTick;
            gImuSmState = IMU_SM_WAIT;
        } else {
            gSensorSnapshot.error_flags |= SENSOR_ERR_IMU_BUS;
            gSensorSnapshot.imu.valid = 0U;
            Sensors_DiagHwError(&gSensorsDebug.imu, nowMs);
            gImuNextDueMs = nowMs + SENSORS_IMU_PERIOD_MS;
        }
        return;

    case IMU_SM_WAIT:
        if (SensorsImuHook_PollRead(&sample)) {
            gSensorSnapshot.imu.ax = sample.ax;
            gSensorSnapshot.imu.ay = sample.ay;
            gSensorSnapshot.imu.az = sample.az;
            gSensorSnapshot.imu.gx = sample.gx;
            gSensorSnapshot.imu.gy = sample.gy;
            gSensorSnapshot.imu.gz = sample.gz;
            gSensorSnapshot.imu.pitch_cdeg = sample.pitch_cdeg;
            gSensorSnapshot.imu.yaw_rate_cdeg_s = sample.yaw_rate_cdeg_s;
            gSensorSnapshot.imu.valid = 1U;
            gSensorSnapshot.error_flags &= ~(SENSOR_ERR_IMU_TIMEOUT | SENSOR_ERR_IMU_BUS);
            Sensors_DiagOk(&gSensorsDebug.imu, nowMs);

            gImuSmState = IMU_SM_IDLE;
            gImuNextDueMs = nowMs + SENSORS_IMU_PERIOD_MS;
            return;
        }

        elapsedUs = Sensors_TimerDeltaUs(gImuSmStartTick, nowTick);
        if (elapsedUs > SENSORS_IMU_TIMEOUT_US) {
            SensorsImuHook_AbortRead();
            gSensorSnapshot.error_flags |= SENSOR_ERR_IMU_TIMEOUT;
            gSensorSnapshot.imu.valid = 0U;
            Sensors_DiagTimeout(&gSensorsDebug.imu, nowMs);

            gImuSmState = IMU_SM_IDLE;
            gImuNextDueMs = nowMs + SENSORS_IMU_PERIOD_MS;
        }
        return;

    default:
        gImuSmState = IMU_SM_IDLE;
        return;
    }
#else
    (void)nowMs;
    (void)nowTick;
#endif
}

static uint16_t Sensors_AgeMsFromDiag(const SensorDiag *diag, uint32_t nowMs)
{
    uint32_t age;

    if (diag->ok_count == 0U) {
        return 0xFFFFU;
    }

    age = nowMs - diag->last_ok_ms;
    if (age > 0xFFFFU) {
        age = 0xFFFFU;
    }

    return (uint16_t)age;
}

static void Sensors_UpdateStaleCounters(uint32_t nowMs)
{
    uint16_t age;
    uint32_t stale = 0U;

    age = Sensors_AgeMsFromDiag((const SensorDiag *)&gSensorsDebug.line, nowMs);
    gSensorSnapshot.line.age_ms = age;
    if ((age > SENSORS_LINE_STALE_MS) || (gSensorSnapshot.line.valid == 0U)) {
        stale |= SENSOR_STALE_LINE;
        gSensorSnapshot.line.valid = 0U;
        Sensors_DiagTrackStale(&gSensorsDebug.line, 1U, &gStaleLine);
    } else {
        Sensors_DiagTrackStale(&gSensorsDebug.line, 0U, &gStaleLine);
    }
    if (age > gSensorsDebug.line.max_age_ms) {
        gSensorsDebug.line.max_age_ms = age;
    }

    age = Sensors_AgeMsFromDiag((const SensorDiag *)&gSensorsDebug.us_left, nowMs);
    gSensorSnapshot.us.left_age_ms = age;
    if ((age > SENSORS_US_STALE_MS) || (gSensorSnapshot.us.left_valid == 0U)) {
        stale |= SENSOR_STALE_US_LEFT;
        gSensorSnapshot.us.left_valid = 0U;
        Sensors_DiagTrackStale(&gSensorsDebug.us_left, 1U, &gStaleUsLeft);
    } else {
        Sensors_DiagTrackStale(&gSensorsDebug.us_left, 0U, &gStaleUsLeft);
    }
    if (age > gSensorsDebug.us_left.max_age_ms) {
        gSensorsDebug.us_left.max_age_ms = age;
    }

    age = Sensors_AgeMsFromDiag((const SensorDiag *)&gSensorsDebug.us_right, nowMs);
    gSensorSnapshot.us.right_age_ms = age;
    if ((age > SENSORS_US_STALE_MS) || (gSensorSnapshot.us.right_valid == 0U)) {
        stale |= SENSOR_STALE_US_RIGHT;
        gSensorSnapshot.us.right_valid = 0U;
        Sensors_DiagTrackStale(&gSensorsDebug.us_right, 1U, &gStaleUsRight);
    } else {
        Sensors_DiagTrackStale(&gSensorsDebug.us_right, 0U, &gStaleUsRight);
    }
    if (age > gSensorsDebug.us_right.max_age_ms) {
        gSensorsDebug.us_right.max_age_ms = age;
    }

    age = Sensors_AgeMsFromDiag((const SensorDiag *)&gSensorsDebug.ir, nowMs);
    gSensorSnapshot.ir.age_ms = age;
    if ((age > SENSORS_IR_STALE_MS) || (gSensorSnapshot.ir.valid == 0U)) {
        stale |= SENSOR_STALE_IR;
        gSensorSnapshot.ir.valid = 0U;
        Sensors_DiagTrackStale(&gSensorsDebug.ir, 1U, &gStaleIr);
    } else {
        Sensors_DiagTrackStale(&gSensorsDebug.ir, 0U, &gStaleIr);
    }
    if (age > gSensorsDebug.ir.max_age_ms) {
        gSensorsDebug.ir.max_age_ms = age;
    }

    age = Sensors_AgeMsFromDiag((const SensorDiag *)&gSensorsDebug.imu, nowMs);
    gSensorSnapshot.imu.age_ms = age;
    if ((age > SENSORS_IMU_STALE_MS) || (gSensorSnapshot.imu.valid == 0U)) {
        stale |= SENSOR_STALE_IMU;
        gSensorSnapshot.imu.valid = 0U;
        Sensors_DiagTrackStale(&gSensorsDebug.imu, 1U, &gStaleImu);
    } else {
        Sensors_DiagTrackStale(&gSensorsDebug.imu, 0U, &gStaleImu);
    }
    if (age > gSensorsDebug.imu.max_age_ms) {
        gSensorsDebug.imu.max_age_ms = age;
    }

    age = Sensors_AgeMsFromDiag((const SensorDiag *)&gSensorsDebug.bump, nowMs);
    gSensorSnapshot.bump.age_ms = age;
    if ((age > SENSORS_BUMP_STALE_MS) || (gSensorSnapshot.bump.valid == 0U)) {
        stale |= SENSOR_STALE_BUMP;
        gSensorSnapshot.bump.valid = 0U;
        Sensors_DiagTrackStale(&gSensorsDebug.bump, 1U, &gStaleBump);
    } else {
        Sensors_DiagTrackStale(&gSensorsDebug.bump, 0U, &gStaleBump);
    }
    if (age > gSensorsDebug.bump.max_age_ms) {
        gSensorsDebug.bump.max_age_ms = age;
    }

    gSensorSnapshot.stale_flags = stale;
}

static void Sensors_DiagKick(volatile SensorDiag *diag)
{
    diag->kick_count++;
}

static void Sensors_DiagOk(volatile SensorDiag *diag, uint32_t nowMs)
{
    if (diag->consecutive_fail != 0U) {
        diag->recover_count++;
    }
    diag->ok_count++;
    diag->consecutive_fail = 0U;
    diag->last_ok_ms = nowMs;
}

static void Sensors_DiagTimeout(volatile SensorDiag *diag, uint32_t nowMs)
{
    diag->timeout_count++;
    if (diag->consecutive_fail < 0xFFFFU) {
        diag->consecutive_fail++;
    }
    diag->last_err_ms = nowMs;
}

#if SENSORS_ENABLE_IMU_HOOK
static void Sensors_DiagHwError(volatile SensorDiag *diag, uint32_t nowMs)
{
    diag->hw_err_count++;
    if (diag->consecutive_fail < 0xFFFFU) {
        diag->consecutive_fail++;
    }
    diag->last_err_ms = nowMs;
}
#endif

static void Sensors_DiagTrackStale(volatile SensorDiag *diag, uint8_t stale, uint8_t *prevStale)
{
    if ((stale != 0U) && (*prevStale == 0U)) {
        diag->stale_enter_count++;
    }
    *prevStale = stale;
}
