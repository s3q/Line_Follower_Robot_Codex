#include "zone_detector.h"
#include "mission_config.h"

typedef struct {
    int mean;
    int spread;
    int mad;
    int lrImbalance;
    int centerOuter;
} ZoneFeatures;

volatile uint8_t gZoneLastMask = 0U;
volatile uint8_t gZoneRawCount = 0U;
volatile ZoneId gZoneClassified = ZONE_UNKNOWN;
volatile uint16_t gZoneConfidence = 0U;
volatile uint16_t gZoneBestScore = 0U;
volatile uint16_t gZoneSecondScore = 0U;
volatile uint8_t gZoneDebounceCount = 0U;
volatile uint16_t gZoneLockoutTicks = 0U;
volatile int gZoneFeatMean = 0;
volatile int gZoneFeatSpread = 0;
volatile int gZoneFeatMad = 0;
volatile int gZoneFeatLrImbalance = 0;
volatile int gZoneFeatCenterOuter = 0;

static uint8_t gZoneHistory[MISSION_ZONE_WINDOW_N] = {0U};
static uint8_t gZoneHistIndex = 0U;
static uint8_t gZoneHistCount = 0U;
static uint8_t gZoneChannelAcc[8] = {0U};
static ZoneId gZoneDebounceCandidate = ZONE_UNKNOWN;
static ZoneId gZoneLastSwitchedZone = ZONE_UNKNOWN;

static uint16_t ZoneDetector_AbsDiff(int a, int b);
static void ZoneDetector_UpdateWindow(uint8_t lineMask);
static void ZoneDetector_ExtractFeatures(ZoneFeatures *feat);
static uint16_t ZoneDetector_ScoreForZone(ZoneId zone, const ZoneFeatures *feat);
static ZoneId ZoneDetector_Classify(const ZoneFeatures *feat, uint16_t *confidenceOut);

static uint16_t ZoneDetector_AbsDiff(int a, int b)
{
    int d = a - b;
    if (d < 0) {
        d = -d;
    }
    return (uint16_t)d;
}

static void ZoneDetector_UpdateWindow(uint8_t lineMask)
{
    uint8_t oldMask;
    uint8_t bit;

    oldMask = gZoneHistory[gZoneHistIndex];
    gZoneHistory[gZoneHistIndex] = lineMask;
    gZoneHistIndex++;
    if (gZoneHistIndex >= MISSION_ZONE_WINDOW_N) {
        gZoneHistIndex = 0U;
    }

    if (gZoneHistCount < MISSION_ZONE_WINDOW_N) {
        gZoneHistCount++;
    }

    for (bit = 0U; bit < 8U; bit++) {
        uint8_t mask = (uint8_t)(1U << bit);
        if ((oldMask & mask) != 0U) {
            if (gZoneChannelAcc[bit] > 0U) {
                gZoneChannelAcc[bit]--;
            }
        }
        if ((lineMask & mask) != 0U) {
            gZoneChannelAcc[bit]++;
        }
    }
}

static void ZoneDetector_ExtractFeatures(ZoneFeatures *feat)
{
    uint8_t i;
    int sum;
    int mean;
    int minv;
    int maxv;
    int madSum;
    int left;
    int right;
    int center;
    int outer;

    sum = 0;
    minv = 255;
    maxv = 0;
    for (i = 0U; i < 8U; i++) {
        int v = gZoneChannelAcc[i];
        sum += v;
        if (v < minv) {
            minv = v;
        }
        if (v > maxv) {
            maxv = v;
        }
    }

    mean = (sum + 4) >> 3;
    madSum = 0;
    for (i = 0U; i < 8U; i++) {
        int d = gZoneChannelAcc[i] - mean;
        if (d < 0) {
            d = -d;
        }
        madSum += d;
    }

    left = gZoneChannelAcc[0] + gZoneChannelAcc[1] + gZoneChannelAcc[2] + gZoneChannelAcc[3];
    right = gZoneChannelAcc[4] + gZoneChannelAcc[5] + gZoneChannelAcc[6] + gZoneChannelAcc[7];
    center = gZoneChannelAcc[3] + gZoneChannelAcc[4];
    outer = gZoneChannelAcc[0] + gZoneChannelAcc[1] + gZoneChannelAcc[6] + gZoneChannelAcc[7];

    feat->mean = mean;
    feat->spread = maxv - minv;
    feat->mad = (madSum + 4) >> 3;
    feat->lrImbalance = (left > right) ? (left - right) : (right - left);
    feat->centerOuter = ((center << 1) > outer) ? ((center << 1) - outer) : (outer - (center << 1));
}

static uint16_t ZoneDetector_ScoreForZone(ZoneId zone, const ZoneFeatures *feat)
{
    int cm;
    int cs;
    int cd;
    int cl;
    int co;
    uint32_t score;

    switch (zone) {
    case ZONE_GREEN:
        cm = ZONE_GREEN_MEAN; cs = ZONE_GREEN_SPREAD; cd = ZONE_GREEN_MAD;
        cl = ZONE_GREEN_LR; co = ZONE_GREEN_CO;
        break;
    case ZONE_YELLOW:
        cm = ZONE_YELLOW_MEAN; cs = ZONE_YELLOW_SPREAD; cd = ZONE_YELLOW_MAD;
        cl = ZONE_YELLOW_LR; co = ZONE_YELLOW_CO;
        break;
    case ZONE_BLUE:
        cm = ZONE_BLUE_MEAN; cs = ZONE_BLUE_SPREAD; cd = ZONE_BLUE_MAD;
        cl = ZONE_BLUE_LR; co = ZONE_BLUE_CO;
        break;
    case ZONE_RED:
        cm = ZONE_RED_MEAN; cs = ZONE_RED_SPREAD; cd = ZONE_RED_MAD;
        cl = ZONE_RED_LR; co = ZONE_RED_CO;
        break;
    default:
        return 0xFFFFU;
    }

    score = 0U;
    score += (uint32_t)MISSION_ZONE_W_MEAN * ZoneDetector_AbsDiff(feat->mean, cm);
    score += (uint32_t)MISSION_ZONE_W_SPREAD * ZoneDetector_AbsDiff(feat->spread, cs);
    score += (uint32_t)MISSION_ZONE_W_MAD * ZoneDetector_AbsDiff(feat->mad, cd);
    score += (uint32_t)MISSION_ZONE_W_LR * ZoneDetector_AbsDiff(feat->lrImbalance, cl);
    score += (uint32_t)MISSION_ZONE_W_CENTER_OUTER * ZoneDetector_AbsDiff(feat->centerOuter, co);

    if (score > 0xFFFFU) {
        score = 0xFFFFU;
    }
    return (uint16_t)score;
}

static ZoneId ZoneDetector_Classify(const ZoneFeatures *feat, uint16_t *confidenceOut)
{
    ZoneId zones[4] = {ZONE_GREEN, ZONE_YELLOW, ZONE_BLUE, ZONE_RED};
    ZoneId bestZone;
    uint16_t best;
    uint16_t second;
    uint8_t i;
    uint16_t conf;

    *confidenceOut = 0U;

    if ((feat->mean < MISSION_ZONE_MEAN_MIN) ||
        (feat->mean > MISSION_ZONE_MEAN_MAX) ||
        (feat->spread > MISSION_ZONE_SPREAD_MAX) ||
        (feat->mad > MISSION_ZONE_MAD_MAX)) {
        gZoneBestScore = 0xFFFFU;
        gZoneSecondScore = 0xFFFFU;
        return ZONE_UNKNOWN;
    }

    bestZone = ZONE_UNKNOWN;
    best = 0xFFFFU;
    second = 0xFFFFU;

    for (i = 0U; i < 4U; i++) {
        uint16_t s = ZoneDetector_ScoreForZone(zones[i], feat);
        if (s < best) {
            second = best;
            best = s;
            bestZone = zones[i];
        } else if (s < second) {
            second = s;
        }
    }

    gZoneBestScore = best;
    gZoneSecondScore = second;

    if (best > MISSION_ZONE_SCORE_MAX_ACCEPT) {
        return ZONE_UNKNOWN;
    }

    if ((second - best) < MISSION_ZONE_SCORE_MARGIN_MIN) {
        return ZONE_UNKNOWN;
    }

    {
        uint16_t confByScore;
        uint16_t confByMargin;
        uint16_t margin = (uint16_t)(second - best);

        confByScore = (uint16_t)(1000U - ((uint32_t)best * 1000U) / MISSION_ZONE_SCORE_MAX_ACCEPT);
        if (confByScore > 1000U) {
            confByScore = 0U;
        }
        confByMargin = (uint16_t)(((uint32_t)margin * 1000U) / MISSION_ZONE_SCORE_MARGIN_TGT);
        if (confByMargin > 1000U) {
            confByMargin = 1000U;
        }

        conf = (confByScore < confByMargin) ? confByScore : confByMargin;
    }

    *confidenceOut = conf;
    if (conf < MISSION_ZONE_CONF_REPORT_MIN) {
        return ZONE_UNKNOWN;
    }

    return bestZone;
}

void ZoneDetector_Init(void)
{
    uint8_t i;
    gZoneLastMask = 0U;
    gZoneRawCount = 0U;
    gZoneClassified = ZONE_UNKNOWN;
    gZoneConfidence = 0U;
    gZoneBestScore = 0U;
    gZoneSecondScore = 0U;
    gZoneDebounceCount = 0U;
    gZoneLockoutTicks = 0U;
    gZoneFeatMean = 0;
    gZoneFeatSpread = 0;
    gZoneFeatMad = 0;
    gZoneFeatLrImbalance = 0;
    gZoneFeatCenterOuter = 0;
    gZoneHistIndex = 0U;
    gZoneHistCount = 0U;
    gZoneDebounceCandidate = ZONE_UNKNOWN;
    gZoneLastSwitchedZone = ZONE_UNKNOWN;

    for (i = 0U; i < MISSION_ZONE_WINDOW_N; i++) {
        gZoneHistory[i] = 0U;
    }
    for (i = 0U; i < 8U; i++) {
        gZoneChannelAcc[i] = 0U;
    }
}

ZoneDecision ZoneDetector_Update(uint8_t lineMask)
{
    ZoneDecision out;
    ZoneFeatures feat;
    ZoneId z;
    uint16_t conf;
    uint8_t countBits;

    out.zone = ZONE_UNKNOWN;
    out.confidence = 0U;
    out.switchQualified = 0U;

    gZoneLastMask = lineMask;

    countBits = 0U;
    {
        uint8_t m = lineMask;
        while (m != 0U) {
            countBits += (uint8_t)(m & 1U);
            m >>= 1;
        }
    }
    gZoneRawCount = countBits;

    ZoneDetector_UpdateWindow(lineMask);

    if (gZoneLockoutTicks > 0U) {
        gZoneLockoutTicks--;
    }

    if (gZoneHistCount < MISSION_ZONE_WINDOW_N) {
        gZoneClassified = ZONE_UNKNOWN;
        gZoneConfidence = 0U;
        return out;
    }

    ZoneDetector_ExtractFeatures(&feat);
    gZoneFeatMean = feat.mean;
    gZoneFeatSpread = feat.spread;
    gZoneFeatMad = feat.mad;
    gZoneFeatLrImbalance = feat.lrImbalance;
    gZoneFeatCenterOuter = feat.centerOuter;

    z = ZoneDetector_Classify(&feat, &conf);
    gZoneClassified = z;
    gZoneConfidence = conf;

    out.zone = z;
    out.confidence = conf;

    if ((z == ZONE_UNKNOWN) || (conf < MISSION_ZONE_CONF_SWITCH_MIN)) {
        gZoneDebounceCandidate = ZONE_UNKNOWN;
        gZoneDebounceCount = 0U;
        return out;
    }

    if (z != gZoneDebounceCandidate) {
        gZoneDebounceCandidate = z;
        gZoneDebounceCount = 1U;
    } else if (gZoneDebounceCount < 255U) {
        gZoneDebounceCount++;
    }

    if (gZoneDebounceCount < MISSION_ZONE_DEBOUNCE_N) {
        return out;
    }

    if (gZoneLockoutTicks > 0U) {
        return out;
    }

    if (z == gZoneLastSwitchedZone) {
        return out;
    }

    gZoneLastSwitchedZone = z;
    gZoneLockoutTicks = MISSION_ZONE_LOCKOUT_TICKS;
    gZoneDebounceCount = 0U;
    gZoneDebounceCandidate = ZONE_UNKNOWN;
    out.switchQualified = 1U;
    return out;
}
