#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include <stdint.h>

typedef struct {
    uint8_t latestLineReading;
    uint8_t latestBumpState;
    uint16_t lineWhiteCounts[8];
    uint16_t lineBlackCounts[8];
    uint16_t lineThresholdCounts[8];
    float ultrasonicLeftBaselineCm;
    float ultrasonicRightBaselineCm;
} CalibrationData;

extern volatile CalibrationData gCalibrationData;

void Calibration_Init(void);
void Calibration_Run(void);

#endif /* CALIBRATION_H_ */
