# Reflectance Zone Detector (P7.0-P7.7 only)

## 1) Feature Set (integer friendly)

Input per cycle: `raw8[8]` (or derived 8-bit line mask from current reflectance driver).

Compute over a rolling window of `N` frames:
- `mean`: average channel activity across 8 sensors.
- `spread`: `max(channel_count) - min(channel_count)`.
- `mad`: mean absolute deviation from `mean`.
- `lr_imbalance`: `abs((ch0+ch1+ch2+ch3) - (ch4+ch5+ch6+ch7))`.
- `center_outer`: `abs(2*(ch3+ch4) - (ch0+ch1+ch6+ch7))`.

All features are integer; no floating point required.

---

## 2) Calibration Procedure

1. Prepare one physical patch for each zone label:
   - GREEN, YELLOW, BLUE, RED (your own floor materials/markers).
2. Hold robot centered on one zone and log 200-500 cycles:
   - raw mask
   - extracted features (`mean`, `spread`, `mad`, `lr_imbalance`, `center_outer`)
3. Repeat for all zones.
4. For each zone, compute robust center values (median recommended) for each feature.
5. Set those medians as zone centroids in config.
6. Set score max + margin thresholds:
   - `SCORE_MAX_ACCEPT`: slightly above good in-zone scores
   - `SCORE_MARGIN_MIN`: minimum separation between best and second-best zone
7. Validate mixed runs and tune:
   - If false switches occur: raise debounce, lockout, and confidence threshold.
   - If missed switches occur: lower confidence threshold slightly or relax score limit.

---

## 3) Pseudocode: `classify_zone(raw8[]) -> {zone, confidence}`

```text
function classify_zone(raw8[8]):
    update_window(raw8)                     // rolling N-frame accumulator
    if window_not_full:
        return {UNKNOWN, 0}

    feat = extract_features_from_window()

    if feat violates hard gates:
        return {UNKNOWN, 0}

    best_zone = UNKNOWN
    best_score = INF
    second_score = INF

    for zone in [GREEN, YELLOW, BLUE, RED]:
        score = weighted_distance(feat, centroid[zone])
        if score < best_score:
            second_score = best_score
            best_score = score
            best_zone = zone
        else if score < second_score:
            second_score = score

    if best_score > SCORE_MAX_ACCEPT:
        return {UNKNOWN, 0}

    margin = second_score - best_score
    if margin < SCORE_MARGIN_MIN:
        return {UNKNOWN, 0}

    conf_score = 1000 - (best_score * 1000 / SCORE_MAX_ACCEPT)
    conf_margin = min(1000, margin * 1000 / SCORE_MARGIN_TARGET)
    confidence = min(conf_score, conf_margin)

    if confidence < CONF_REPORT_MIN:
        return {UNKNOWN, confidence}

    // debounce + lockout gating for switching decision
    if zone != debounce_candidate:
        debounce_candidate = zone
        debounce_count = 1
        return {zone, confidence, switchQualified=0}

    debounce_count++
    if debounce_count < DEBOUNCE_N:
        return {zone, confidence, switchQualified=0}

    if lockout_ticks > 0:
        return {zone, confidence, switchQualified=0}

    if confidence < CONF_SWITCH_MIN:
        return {zone, confidence, switchQualified=0}

    if zone == last_switched_zone:
        return {zone, confidence, switchQualified=0}

    last_switched_zone = zone
    lockout_ticks = LOCKOUT_TICKS
    debounce_count = 0
    debounce_candidate = UNKNOWN
    return {zone, confidence, switchQualified=1}
```

---

## 4) Recommended Defaults + Tuning

Suggested starting defaults:
- `WINDOW_N = 8`
- `DEBOUNCE_N = 5`
- `LOCKOUT_TICKS = 120`
- `CONF_REPORT_MIN = 450`
- `CONF_SWITCH_MIN = 700`
- `SCORE_MAX_ACCEPT = 90`
- `SCORE_MARGIN_MIN = 8`
- `SCORE_MARGIN_TARGET = 24`

Tuning:
- Too many false switches:
  - increase `DEBOUNCE_N`
  - increase `CONF_SWITCH_MIN`
  - increase `SCORE_MARGIN_MIN`
  - increase `LOCKOUT_TICKS`
- Too slow to switch:
  - decrease `DEBOUNCE_N` slightly
  - decrease `CONF_SWITCH_MIN` slightly
  - lower `SCORE_MAX_ACCEPT` only if true samples are currently rejected by poor centroid fit
- Unstable classification between two zones:
  - improve centroid calibration data quality
  - adjust feature weights toward the features with strongest real separation

