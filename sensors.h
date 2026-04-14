#ifndef SENSORS_H_
#define SENSORS_H_

#include <stdint.h>

/*
 * Scheduler defaults:
 * - control/snapshot tick at 5 ms (200 Hz)
 * - slower tasks are decimated inside Sensors_Update()
 */
#ifndef SENSORS_CONTROL_PERIOD_MS
#define SENSORS_CONTROL_PERIOD_MS                5U
#endif

#ifndef SENSORS_LINE_PERIOD_MS
#define SENSORS_LINE_PERIOD_MS                   5U
#endif

#ifndef SENSORS_BUMP_PERIOD_MS
#define SENSORS_BUMP_PERIOD_MS                   5U
#endif

#ifndef SENSORS_IR_PERIOD_MS
#define SENSORS_IR_PERIOD_MS                     5U
#endif

#ifndef SENSORS_IMU_PERIOD_MS
#define SENSORS_IMU_PERIOD_MS                   10U
#endif

#ifndef SENSORS_ULTRA_PERIOD_MS
#define SENSORS_ULTRA_PERIOD_MS                 25U
#endif

/*
 * Per-sensor transaction timeouts.
 */
#ifndef SENSORS_LINE_TIMEOUT_US
#define SENSORS_LINE_TIMEOUT_US               2500U
#endif

#ifndef SENSORS_US_START_TIMEOUT_US
#define SENSORS_US_START_TIMEOUT_US           2000U
#endif

#ifndef SENSORS_US_ECHO_TIMEOUT_US
#define SENSORS_US_ECHO_TIMEOUT_US           15000U
#endif

#ifndef SENSORS_IR_TIMEOUT_US
#define SENSORS_IR_TIMEOUT_US                  300U
#endif

#ifndef SENSORS_IMU_TIMEOUT_US
#define SENSORS_IMU_TIMEOUT_US                2000U
#endif

/*
 * Stale thresholds for snapshot health.
 */
#ifndef SENSORS_LINE_STALE_MS
#define SENSORS_LINE_STALE_MS                   30U
#endif

#ifndef SENSORS_US_STALE_MS
#define SENSORS_US_STALE_MS                    120U
#endif

#ifndef SENSORS_IR_STALE_MS
#define SENSORS_IR_STALE_MS                     50U
#endif

#ifndef SENSORS_IMU_STALE_MS
#define SENSORS_IMU_STALE_MS                    50U
#endif

#ifndef SENSORS_BUMP_STALE_MS
#define SENSORS_BUMP_STALE_MS                   20U
#endif

/*
 * Reflectance sampling time for RC discharge.
 */
#ifndef SENSORS_LINE_SAMPLE_US
#define SENSORS_LINE_SAMPLE_US                1000U
#endif

/*
 * Optional IMU hook:
 * 0 = IMU logic disabled (snapshot reports IMU stale/invalid)
 * 1 = use weak hook functions in sensors.c (override from MPU6500 module)
 */
#ifndef SENSORS_ENABLE_IMU_HOOK
#define SENSORS_ENABLE_IMU_HOOK                 0U
#endif

/*
 * Bump debounce in scheduler samples.
 */
#ifndef SENSORS_BUMP_DEBOUNCE_SAMPLES
#define SENSORS_BUMP_DEBOUNCE_SAMPLES           2U
#endif

/*
 * Error flags (latched until successful update of the same sensor channel).
 */
#define SENSOR_ERR_LINE_TIMEOUT           (1UL << 0)
#define SENSOR_ERR_ULTRA_LEFT_TIMEOUT     (1UL << 1)
#define SENSOR_ERR_ULTRA_RIGHT_TIMEOUT    (1UL << 2)
#define SENSOR_ERR_IR_TIMEOUT             (1UL << 3)
#define SENSOR_ERR_IMU_TIMEOUT            (1UL << 4)
#define SENSOR_ERR_IMU_BUS                (1UL << 5)

/*
 * Stale flags (computed each update from age vs stale threshold).
 */
#define SENSOR_STALE_LINE                 (1UL << 0)
#define SENSOR_STALE_US_LEFT              (1UL << 1)
#define SENSOR_STALE_US_RIGHT             (1UL << 2)
#define SENSOR_STALE_IR                   (1UL << 3)
#define SENSOR_STALE_IMU                  (1UL << 4)
#define SENSOR_STALE_BUMP                 (1UL << 5)

typedef struct {
    uint32_t kick_count;
    uint32_t ok_count;
    uint32_t timeout_count;
    uint32_t hw_err_count;
    uint32_t stale_enter_count;
    uint32_t recover_count;
    uint16_t max_age_ms;
    uint16_t consecutive_fail;
    uint32_t last_ok_ms;
    uint32_t last_err_ms;
} SensorDiag;

typedef struct {
    SensorDiag line;
    SensorDiag us_left;
    SensorDiag us_right;
    SensorDiag ir;
    SensorDiag imu;
    SensorDiag bump;
    uint32_t loop_overrun_count;
    uint32_t loop_missed_deadline_count;
} SensorsDebug;

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t pitch_cdeg;
    int16_t yaw_rate_cdeg_s;
} SensorImuSample;

typedef struct {
    uint32_t ts_ms;
    uint32_t seq;

    struct {
        uint8_t mask;
        uint8_t count;
        int16_t line_err;
        uint8_t valid;
        uint16_t age_ms;
    } line;

    struct {
        uint16_t left_mm;
        uint16_t right_mm;
        uint8_t left_valid;
        uint8_t right_valid;
        uint16_t left_age_ms;
        uint16_t right_age_ms;
    } us;

    struct {
        uint16_t raw;
        uint16_t dist_mm;
        uint8_t valid;
        uint16_t age_ms;
    } ir;

    struct {
        int16_t ax;
        int16_t ay;
        int16_t az;
        int16_t gx;
        int16_t gy;
        int16_t gz;
        int16_t pitch_cdeg;
        int16_t yaw_rate_cdeg_s;
        uint8_t valid;
        uint16_t age_ms;
    } imu;

    struct {
        uint8_t raw_mask;
        uint8_t debounced_mask;
        uint8_t valid;
        uint16_t age_ms;
    } bump;

    uint32_t error_flags;
    uint32_t stale_flags;
} SensorSnapshot;

extern volatile SensorSnapshot gSensorSnapshot;
extern volatile SensorsDebug gSensorsDebug;

void Sensors_Init(void);
void Sensors_Update(uint32_t now_ms);
void Sensors_ClearErrors(void);
const volatile SensorSnapshot *Sensors_GetSnapshot(void);
const volatile SensorsDebug *Sensors_GetDebug(void);

#endif /* SENSORS_H_ */
