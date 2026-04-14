#ifndef ZONE_DETECTOR_H_
#define ZONE_DETECTOR_H_

#include <stdint.h>

typedef enum {
    ZONE_UNKNOWN = 0,
    ZONE_GREEN = 1,
    ZONE_YELLOW = 2,
    ZONE_BLUE = 3,
    ZONE_RED = 4
} ZoneId;

typedef struct {
    ZoneId zone;
    uint16_t confidence;       /* 0..1000 */
    uint8_t switchQualified;   /* 1 when debounce+confidence+lockout allow switch */
} ZoneDecision;

extern volatile uint8_t gZoneLastMask;
extern volatile uint8_t gZoneRawCount;
extern volatile ZoneId gZoneClassified;
extern volatile uint16_t gZoneConfidence;
extern volatile uint16_t gZoneBestScore;
extern volatile uint16_t gZoneSecondScore;
extern volatile uint8_t gZoneDebounceCount;
extern volatile uint16_t gZoneLockoutTicks;
extern volatile int gZoneFeatMean;
extern volatile int gZoneFeatSpread;
extern volatile int gZoneFeatMad;
extern volatile int gZoneFeatLrImbalance;
extern volatile int gZoneFeatCenterOuter;

void ZoneDetector_Init(void);
ZoneDecision ZoneDetector_Update(uint8_t lineMask);

#endif /* ZONE_DETECTOR_H_ */
