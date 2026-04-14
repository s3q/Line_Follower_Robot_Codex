#ifndef MPU6500_H_
#define MPU6500_H_

#include <stdint.h>

extern volatile uint8_t gMpu6500Present;
extern volatile uint8_t gMpu6500Calibrated;
extern volatile uint8_t gMpu6500Still;
extern volatile uint8_t gMpu6500Motion;
extern volatile uint8_t gMpu6500Fault;
extern volatile int16_t gMpu6500AxRaw;
extern volatile int16_t gMpu6500AyRaw;
extern volatile int16_t gMpu6500AzRaw;
extern volatile int16_t gMpu6500GxRaw;
extern volatile int16_t gMpu6500GyRaw;
extern volatile int16_t gMpu6500GzRaw;
extern volatile int16_t gMpu6500TempRaw;
extern volatile int16_t gMpu6500AxCorr;
extern volatile int16_t gMpu6500AyCorr;
extern volatile int16_t gMpu6500AzCorr;
extern volatile int16_t gMpu6500GxCorr;
extern volatile int16_t gMpu6500GyCorr;
extern volatile int16_t gMpu6500GzCorr;
extern volatile int16_t gMpu6500GyroBiasX;
extern volatile int16_t gMpu6500GyroBiasY;
extern volatile int16_t gMpu6500GyroBiasZ;
extern volatile int16_t gMpu6500AccelBiasX;
extern volatile int16_t gMpu6500AccelBiasY;
extern volatile int16_t gMpu6500AccelBiasZ;
extern volatile int gMpu6500PitchDeg;
extern volatile int gMpu6500RollDeg;
extern volatile int gMpu6500YawRateDps;
extern volatile int gMpu6500YawDeltaDeg;
extern volatile int gMpu6500TempC;

void Mpu6500_Init(void);
uint8_t Mpu6500_Calibrate(void);
void Mpu6500_Update(void);
void Mpu6500_ResetYawDelta(void);

#endif /* MPU6500_H_ */
