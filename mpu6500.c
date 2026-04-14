#include "mpu6500.h"
#include "robot_config.h"
#include "robot_pins.h"
#include <stdbool.h>
#include <stdint.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#define MPU6500_REG_SMPLRT_DIV              0x19U
#define MPU6500_REG_CONFIG                  0x1AU
#define MPU6500_REG_GYRO_CONFIG             0x1BU
#define MPU6500_REG_ACCEL_CONFIG            0x1CU
#define MPU6500_REG_ACCEL_CONFIG2           0x1DU
#define MPU6500_REG_PWR_MGMT_1              0x6BU
#define MPU6500_REG_PWR_MGMT_2              0x6CU
#define MPU6500_REG_WHO_AM_I                0x75U
#define MPU6500_REG_ACCEL_XOUT_H            0x3BU
#define MPU6500_WHO_AM_I_VALUE              0x70U

volatile uint8_t gMpu6500Present = 0U;
volatile uint8_t gMpu6500Calibrated = 0U;
volatile uint8_t gMpu6500Still = 0U;
volatile uint8_t gMpu6500Motion = 0U;
volatile uint8_t gMpu6500Fault = 0U;
volatile int16_t gMpu6500AxRaw = 0;
volatile int16_t gMpu6500AyRaw = 0;
volatile int16_t gMpu6500AzRaw = 0;
volatile int16_t gMpu6500GxRaw = 0;
volatile int16_t gMpu6500GyRaw = 0;
volatile int16_t gMpu6500GzRaw = 0;
volatile int16_t gMpu6500TempRaw = 0;
volatile int16_t gMpu6500AxCorr = 0;
volatile int16_t gMpu6500AyCorr = 0;
volatile int16_t gMpu6500AzCorr = 0;
volatile int16_t gMpu6500GxCorr = 0;
volatile int16_t gMpu6500GyCorr = 0;
volatile int16_t gMpu6500GzCorr = 0;
volatile int16_t gMpu6500GyroBiasX = 0;
volatile int16_t gMpu6500GyroBiasY = 0;
volatile int16_t gMpu6500GyroBiasZ = 0;
volatile int16_t gMpu6500AccelBiasX = 0;
volatile int16_t gMpu6500AccelBiasY = 0;
volatile int16_t gMpu6500AccelBiasZ = 0;
volatile int gMpu6500PitchDeg = 0;
volatile int gMpu6500RollDeg = 0;
volatile int gMpu6500YawRateDps = 0;
volatile int gMpu6500YawDeltaDeg = 0;
volatile int gMpu6500TempC = 0;

static bool gMpu6500InitDone = false;

static void Mpu6500_ResetState(void);
static void Mpu6500_DelayMs(uint32_t ms);
static int32_t Mpu6500_Abs32(int32_t value);
static int16_t Mpu6500_CombineBytes(uint8_t msb, uint8_t lsb);
static int32_t Mpu6500_WrapDeg(int32_t angleDeg);
static uint16_t Mpu6500_GetAccel1gCounts(void);
static uint16_t Mpu6500_GetGyroCountsPerDps(void);
static int32_t Mpu6500_ApproxAtan2Deg(int32_t y, int32_t x);
static bool Mpu6500_WriteReg(uint8_t regAddr, uint8_t value);
static bool Mpu6500_ReadRegs(uint8_t regAddr, uint8_t *buffer, uint32_t length);
static bool Mpu6500_ReadSample(
    int16_t *ax,
    int16_t *ay,
    int16_t *az,
    int16_t *temp,
    int16_t *gx,
    int16_t *gy,
    int16_t *gz);

void Mpu6500_Init(void)
{
    eUSCI_I2C_MasterConfig i2cConfig;
    uint8_t whoAmI = 0U;
    uint32_t smclkHz;

    if (gMpu6500InitDone) {
        return;
    }
    Mpu6500_ResetState();

#if !ENABLE_MPU6500_FEATURE
    gMpu6500InitDone = true;
    return;
#else
    GPIO_setAsPeripheralModuleFunctionInputPin(
        MPU6500_I2C_SDA_PORT,
        MPU6500_I2C_SDA_PIN,
        GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(
        MPU6500_I2C_SCL_PORT,
        MPU6500_I2C_SCL_PIN,
        GPIO_PRIMARY_MODULE_FUNCTION);

    smclkHz = CS_getSMCLK();
    if (smclkHz == 0U) {
        smclkHz = 3000000U;
    }

    i2cConfig.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    i2cConfig.i2cClk = smclkHz;
    i2cConfig.dataRate = EUSCI_B_I2C_SET_DATA_RATE_400KBPS;
    i2cConfig.byteCounterThreshold = 0U;
    i2cConfig.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;

    I2C_initMaster(MPU6500_I2C_MODULE, &i2cConfig);
    I2C_setSlaveAddress(MPU6500_I2C_MODULE, MPU6500_I2C_SLAVE_ADDR);
    I2C_enableModule(MPU6500_I2C_MODULE);
    gMpu6500InitDone = true;

    if (!Mpu6500_ReadRegs(MPU6500_REG_WHO_AM_I, &whoAmI, 1U)) {
        gMpu6500Fault = 1U;
        return;
    }

    if (whoAmI != MPU6500_WHO_AM_I_VALUE) {
        gMpu6500Fault = 1U;
        return;
    }

    gMpu6500Present = 1U;

    if (!Mpu6500_WriteReg(MPU6500_REG_PWR_MGMT_1, 0x01U) ||
        !Mpu6500_WriteReg(MPU6500_REG_PWR_MGMT_2, 0x00U) ||
        !Mpu6500_WriteReg(MPU6500_REG_SMPLRT_DIV, MPU6500_SAMPLE_DIV) ||
        !Mpu6500_WriteReg(MPU6500_REG_CONFIG, (uint8_t)(MPU6500_DLPF_CFG & 0x07U)) ||
        !Mpu6500_WriteReg(MPU6500_REG_GYRO_CONFIG, (uint8_t)((MPU6500_GYRO_FS_SEL & 0x03U) << 3)) ||
        !Mpu6500_WriteReg(MPU6500_REG_ACCEL_CONFIG, (uint8_t)((MPU6500_ACCEL_FS_SEL & 0x03U) << 3)) ||
        !Mpu6500_WriteReg(MPU6500_REG_ACCEL_CONFIG2, (uint8_t)(MPU6500_DLPF_CFG & 0x07U))) {
        gMpu6500Fault = 1U;
        gMpu6500Present = 0U;
        return;
    }

    Mpu6500_DelayMs(10U);
#endif
}

uint8_t Mpu6500_Calibrate(void)
{
#if !ENABLE_MPU6500_FEATURE || !ENABLE_MPU6500_CALIBRATION
    gMpu6500Calibrated = 0U;
    return 0U;
#else
    uint32_t sample;
    int32_t sumAx = 0;
    int32_t sumAy = 0;
    int32_t sumAz = 0;
    int32_t sumGx = 0;
    int32_t sumGy = 0;
    int32_t sumGz = 0;
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temp;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    uint16_t accel1g;

    Mpu6500_Init();
    if (gMpu6500Present == 0U) {
        return 0U;
    }

    /*
     * Calibration requires the robot to be still and flat.
     * Gyro bias calibration is mandatory; accel flat calibration is recommended.
     */
    accel1g = Mpu6500_GetAccel1gCounts();
    gMpu6500Calibrated = 0U;

    for (sample = 0U; sample < MPU6500_CALIB_SAMPLES; sample++) {
        if (!Mpu6500_ReadSample(&ax, &ay, &az, &temp, &gx, &gy, &gz)) {
            gMpu6500Fault = 1U;
            return 0U;
        }

        if ((Mpu6500_Abs32(gx) > MPU6500_STILL_GYRO_THR) ||
            (Mpu6500_Abs32(gy) > MPU6500_STILL_GYRO_THR) ||
            (Mpu6500_Abs32(gz) > MPU6500_STILL_GYRO_THR) ||
            ((Mpu6500_Abs32(ax) + Mpu6500_Abs32(ay) + Mpu6500_Abs32((int32_t)az - accel1g)) >
             MPU6500_STILL_ACCEL_THR)) {
            gMpu6500Still = 0U;
            gMpu6500Motion = 1U;
            return 0U;
        }

        sumAx += ax;
        sumAy += ay;
        sumAz += az;
        sumGx += gx;
        sumGy += gy;
        sumGz += gz;
        Mpu6500_DelayMs(2U);
    }

    gMpu6500GyroBiasX = (int16_t)(sumGx / (int32_t)MPU6500_CALIB_SAMPLES);
    gMpu6500GyroBiasY = (int16_t)(sumGy / (int32_t)MPU6500_CALIB_SAMPLES);
    gMpu6500GyroBiasZ = (int16_t)(sumGz / (int32_t)MPU6500_CALIB_SAMPLES);
    gMpu6500AccelBiasX = (int16_t)(sumAx / (int32_t)MPU6500_CALIB_SAMPLES);
    gMpu6500AccelBiasY = (int16_t)(sumAy / (int32_t)MPU6500_CALIB_SAMPLES);
    gMpu6500AccelBiasZ = (int16_t)((sumAz / (int32_t)MPU6500_CALIB_SAMPLES) - (int32_t)accel1g);
    gMpu6500Calibrated = 1U;
    gMpu6500Still = 1U;
    gMpu6500Motion = 0U;
    gMpu6500Fault = 0U;
    gMpu6500YawDeltaDeg = 0;
    gMpu6500PitchDeg = 0;
    gMpu6500RollDeg = 0;
    gMpu6500YawRateDps = 0;

    return 1U;
#endif
}

void Mpu6500_Update(void)
{
#if !ENABLE_MPU6500_FEATURE
    Mpu6500_ResetState();
    return;
#else
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temp;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int32_t accelPitchDeg;
    int32_t accelRollDeg;
    int32_t pitchDeg;
    int32_t rollDeg;
    int32_t yawDelta;
    int32_t yawRateDps;
    int32_t pitchFromGyro;
    int32_t rollFromGyro;
    uint16_t gyroCountsPerDps;
    uint16_t accel1g;
    int32_t accelDeviation;
    int32_t dtMs;

    if (gMpu6500InitDone == false) {
        Mpu6500_Init();
    }
    if (gMpu6500Present == 0U) {
        return;
    }
    if (!Mpu6500_ReadSample(&ax, &ay, &az, &temp, &gx, &gy, &gz)) {
        gMpu6500Fault = 1U;
        return;
    }

    gMpu6500AxRaw = ax;
    gMpu6500AyRaw = ay;
    gMpu6500AzRaw = az;
    gMpu6500GxRaw = gx;
    gMpu6500GyRaw = gy;
    gMpu6500GzRaw = gz;
    gMpu6500TempRaw = temp;

    gMpu6500AxCorr = (int16_t)((int32_t)ax - gMpu6500AccelBiasX);
    gMpu6500AyCorr = (int16_t)((int32_t)ay - gMpu6500AccelBiasY);
    gMpu6500AzCorr = (int16_t)((int32_t)az - gMpu6500AccelBiasZ);
    gMpu6500GxCorr = (int16_t)((int32_t)gx - gMpu6500GyroBiasX);
    gMpu6500GyCorr = (int16_t)((int32_t)gy - gMpu6500GyroBiasY);
    gMpu6500GzCorr = (int16_t)((int32_t)gz - gMpu6500GyroBiasZ);

    accelPitchDeg = Mpu6500_ApproxAtan2Deg(-(int32_t)gMpu6500AxCorr, (int32_t)gMpu6500AzCorr);
    accelRollDeg = Mpu6500_ApproxAtan2Deg((int32_t)gMpu6500AyCorr, (int32_t)gMpu6500AzCorr);

    dtMs = ROBOT_CFG_MAIN_CONTROL_LOOP_MS;
    gyroCountsPerDps = Mpu6500_GetGyroCountsPerDps();
    pitchFromGyro = (int32_t)gMpu6500PitchDeg +
                    (((int32_t)gMpu6500GyCorr * (int32_t)dtMs) /
                     ((int32_t)gyroCountsPerDps * 1000));
    rollFromGyro = (int32_t)gMpu6500RollDeg +
                   (((int32_t)gMpu6500GxCorr * (int32_t)dtMs) /
                    ((int32_t)gyroCountsPerDps * 1000));
    pitchDeg = ((int32_t)MPU6500_PITCH_FILTER_ALPHA * pitchFromGyro +
                (100 - (int32_t)MPU6500_PITCH_FILTER_ALPHA) * accelPitchDeg) / 100;
    rollDeg = ((int32_t)MPU6500_PITCH_FILTER_ALPHA * rollFromGyro +
               (100 - (int32_t)MPU6500_PITCH_FILTER_ALPHA) * accelRollDeg) / 100;
    yawRateDps = ((int32_t)gMpu6500GzCorr >= 0) ?
        (((int32_t)gMpu6500GzCorr + (gyroCountsPerDps / 2U)) / (int32_t)gyroCountsPerDps) :
        (((int32_t)gMpu6500GzCorr - (gyroCountsPerDps / 2U)) / (int32_t)gyroCountsPerDps);

    yawDelta = (int32_t)gMpu6500YawDeltaDeg +
               ((yawRateDps * (int32_t)dtMs) / 1000);
    yawDelta = Mpu6500_WrapDeg(yawDelta);

    gMpu6500PitchDeg = (int)pitchDeg;
    gMpu6500RollDeg = (int)rollDeg;
    gMpu6500YawRateDps = (int)yawRateDps;
    gMpu6500YawDeltaDeg = (int)yawDelta;
    gMpu6500TempC = 21 + (((int32_t)temp - 521) / 340);

    accel1g = Mpu6500_GetAccel1gCounts();
    accelDeviation = Mpu6500_Abs32(gMpu6500AxCorr) +
                     Mpu6500_Abs32(gMpu6500AyCorr) +
                     Mpu6500_Abs32((int32_t)gMpu6500AzCorr - accel1g);
    gMpu6500Still = ((Mpu6500_Abs32(gMpu6500GxCorr) < MPU6500_STILL_GYRO_THR) &&
                     (Mpu6500_Abs32(gMpu6500GyCorr) < MPU6500_STILL_GYRO_THR) &&
                     (Mpu6500_Abs32(gMpu6500GzCorr) < MPU6500_STILL_GYRO_THR) &&
                     (accelDeviation < MPU6500_STILL_ACCEL_THR)) ? 1U : 0U;
#if ENABLE_MPU6500_MOTION_DIAG
    gMpu6500Motion = (gMpu6500Still == 0U) ? 1U : 0U;
#else
    gMpu6500Motion = 0U;
#endif
    gMpu6500Fault = 0U;
#endif
}

void Mpu6500_ResetYawDelta(void)
{
    gMpu6500YawDeltaDeg = 0;
}

static void Mpu6500_ResetState(void)
{
    gMpu6500Present = 0U;
    gMpu6500Calibrated = 0U;
    gMpu6500Still = 0U;
    gMpu6500Motion = 0U;
    gMpu6500Fault = 0U;
    gMpu6500AxRaw = 0;
    gMpu6500AyRaw = 0;
    gMpu6500AzRaw = 0;
    gMpu6500GxRaw = 0;
    gMpu6500GyRaw = 0;
    gMpu6500GzRaw = 0;
    gMpu6500TempRaw = 0;
    gMpu6500AxCorr = 0;
    gMpu6500AyCorr = 0;
    gMpu6500AzCorr = 0;
    gMpu6500GxCorr = 0;
    gMpu6500GyCorr = 0;
    gMpu6500GzCorr = 0;
    gMpu6500GyroBiasX = 0;
    gMpu6500GyroBiasY = 0;
    gMpu6500GyroBiasZ = 0;
    gMpu6500AccelBiasX = 0;
    gMpu6500AccelBiasY = 0;
    gMpu6500AccelBiasZ = 0;
    gMpu6500PitchDeg = 0;
    gMpu6500RollDeg = 0;
    gMpu6500YawRateDps = 0;
    gMpu6500YawDeltaDeg = 0;
    gMpu6500TempC = 0;
}

static void Mpu6500_DelayMs(uint32_t ms)
{
    uint32_t mclkHz;
    uint32_t ticksPerMs;
    uint32_t ticksNeeded;
    uint32_t elapsed;
    uint32_t loadValue;
    uint32_t lastTick;
    uint32_t currentTick;

    if (ms == 0U) {
        return;
    }

    mclkHz = CS_getMCLK();
    if (mclkHz == 0U) {
        mclkHz = 3000000U;
    }

    ticksPerMs = mclkHz / 1000U;
    if (ticksPerMs == 0U) {
        ticksPerMs = 3000U;
    }

    loadValue = SysTick->LOAD + 1U;
    if (loadValue <= 1U) {
        return;
    }

    ticksNeeded = ms * ticksPerMs;
    elapsed = 0U;
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

static int32_t Mpu6500_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int16_t Mpu6500_CombineBytes(uint8_t msb, uint8_t lsb)
{
    return (int16_t)(((uint16_t)msb << 8) | lsb);
}

static int32_t Mpu6500_WrapDeg(int32_t angleDeg)
{
    while (angleDeg > 180) {
        angleDeg -= 360;
    }
    while (angleDeg < -180) {
        angleDeg += 360;
    }

    return angleDeg;
}

static uint16_t Mpu6500_GetAccel1gCounts(void)
{
    switch (MPU6500_ACCEL_FS_SEL) {
    case 1U:
        return 8192U;
    case 2U:
        return 4096U;
    case 3U:
        return 2048U;
    case 0U:
    default:
        return 16384U;
    }
}

static uint16_t Mpu6500_GetGyroCountsPerDps(void)
{
    switch (MPU6500_GYRO_FS_SEL) {
    case 1U:
        return 66U;
    case 2U:
        return 33U;
    case 3U:
        return 16U;
    case 0U:
    default:
        return 131U;
    }
}

static int32_t Mpu6500_ApproxAtan2Deg(int32_t y, int32_t x)
{
    int32_t absY;
    int32_t absX;
    int32_t angle;
    int32_t sum;
    int32_t ratio;

    if ((x == 0) && (y == 0)) {
        return 0;
    }

    absY = Mpu6500_Abs32(y);
    absX = Mpu6500_Abs32(x);
    if (absX >= absY) {
        sum = absX + ((absY * 28) / 100);
        if (sum == 0) {
            angle = 0;
        } else {
            ratio = (absY * 45) / sum;
            angle = ratio;
        }
    } else {
        sum = absY + ((absX * 28) / 100);
        if (sum == 0) {
            angle = 90;
        } else {
            ratio = (absX * 45) / sum;
            angle = 90 - ratio;
        }
    }

    if (x >= 0) {
        if (y >= 0) {
            return angle;
        }
        return -angle;
    }

    if (y >= 0) {
        return 180 - angle;
    }

    return angle - 180;
}

static bool Mpu6500_WriteReg(uint8_t regAddr, uint8_t value)
{
    bool ok;

    I2C_setSlaveAddress(MPU6500_I2C_MODULE, MPU6500_I2C_SLAVE_ADDR);
    ok = I2C_masterSendMultiByteStartWithTimeout(MPU6500_I2C_MODULE, regAddr, 10000U);
    if (!ok) {
        return false;
    }
    ok = I2C_masterSendMultiByteFinishWithTimeout(MPU6500_I2C_MODULE, value, 10000U);

    return ok;
}

static bool Mpu6500_ReadRegs(uint8_t regAddr, uint8_t *buffer, uint32_t length)
{
    uint32_t i;
    uint32_t timeout;

    if ((buffer == 0) || (length == 0U)) {
        return false;
    }

    I2C_setSlaveAddress(MPU6500_I2C_MODULE, MPU6500_I2C_SLAVE_ADDR);
    if (!I2C_masterSendSingleByteWithTimeout(MPU6500_I2C_MODULE, regAddr, 10000U)) {
        return false;
    }

    I2C_masterReceiveStart(MPU6500_I2C_MODULE);

    if (length == 1U) {
        timeout = 10000U;
        while (((I2C_getInterruptStatus(
                     MPU6500_I2C_MODULE,
                     EUSCI_B_I2C_RECEIVE_INTERRUPT0) &
                 EUSCI_B_I2C_RECEIVE_INTERRUPT0) == 0U) &&
               (timeout-- > 0U)) {
        }
        if (timeout == 0U) {
            I2C_masterReceiveMultiByteStop(MPU6500_I2C_MODULE);
            return false;
        }
        buffer[0] = I2C_masterReceiveMultiByteFinish(MPU6500_I2C_MODULE);
        return true;
    }

    for (i = 0U; i < (length - 1U); i++) {
        timeout = 10000U;
        while (((I2C_getInterruptStatus(
                     MPU6500_I2C_MODULE,
                     EUSCI_B_I2C_RECEIVE_INTERRUPT0) &
                 EUSCI_B_I2C_RECEIVE_INTERRUPT0) == 0U) &&
               (timeout-- > 0U)) {
        }
        if (timeout == 0U) {
            I2C_masterReceiveMultiByteStop(MPU6500_I2C_MODULE);
            return false;
        }
        buffer[i] = I2C_masterReceiveMultiByteNext(MPU6500_I2C_MODULE);
    }
    timeout = 10000U;
    while (((I2C_getInterruptStatus(
                 MPU6500_I2C_MODULE,
                 EUSCI_B_I2C_RECEIVE_INTERRUPT0) &
             EUSCI_B_I2C_RECEIVE_INTERRUPT0) == 0U) &&
           (timeout-- > 0U)) {
    }
    if (timeout == 0U) {
        I2C_masterReceiveMultiByteStop(MPU6500_I2C_MODULE);
        return false;
    }
    buffer[length - 1U] = I2C_masterReceiveMultiByteFinish(MPU6500_I2C_MODULE);

    return true;
}

static bool Mpu6500_ReadSample(
    int16_t *ax,
    int16_t *ay,
    int16_t *az,
    int16_t *temp,
    int16_t *gx,
    int16_t *gy,
    int16_t *gz)
{
    uint8_t rawBytes[14];

    if (!Mpu6500_ReadRegs(MPU6500_REG_ACCEL_XOUT_H, rawBytes, sizeof(rawBytes))) {
        return false;
    }

    *ax = Mpu6500_CombineBytes(rawBytes[0], rawBytes[1]);
    *ay = Mpu6500_CombineBytes(rawBytes[2], rawBytes[3]);
    *az = Mpu6500_CombineBytes(rawBytes[4], rawBytes[5]);
    *temp = Mpu6500_CombineBytes(rawBytes[6], rawBytes[7]);
    *gx = Mpu6500_CombineBytes(rawBytes[8], rawBytes[9]);
    *gy = Mpu6500_CombineBytes(rawBytes[10], rawBytes[11]);
    *gz = Mpu6500_CombineBytes(rawBytes[12], rawBytes[13]);

    return true;
}
