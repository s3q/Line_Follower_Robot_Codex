# Mission Controller (TI-RSLK MAX, MSP432)

## 1) Proposed File / Module Layout

### Core mission orchestration
- `mission.h` / `mission.c`
  - Owns top-level mission mode:
    - `LINE_FOLLOW`
    - `OBSTACLE_AVOID`
    - `TORQUE_RAMP`
    - `STOP`
  - Calls existing modules only (`line_follow`, `avoid`, `motor`, etc.).
  - Applies timeout + watchdog safety.

### Mode/profile mapping
- `mode_manager.h` / `mode_manager.c`
  - Maps profile + zone requests to allowed mission modes.
  - Provides boot mode and mode timeout lookup.

### Zone detection/classification
- `zone_detector.h` / `zone_detector.c`
  - Uses only 8-channel reflectance mask (`P7.0..P7.7` via `line.c` API).
  - Produces `{zone, confidence, switchQualified}`.
  - Implements debounce + lockout + low-confidence reject.

### Compile-time configuration
- `mission_config.h`
  - Profile select.
  - Mode enable masks per profile.
  - Watchdog/timeout constants.
  - Zone detector thresholds/weights.

### Existing modules left unchanged
- `motor.c/h` (unchanged)
- `line.c/h` (unchanged)
- `bump.c/h` (unchanged)
- `delay / SysTick` logic (unchanged)

---

## 2) `config.h` Template (Profiles + Key Thresholds)

```c
#ifndef MISSION_CONFIG_H_
#define MISSION_CONFIG_H_

/* Profiles */
#define PROFILE_FULL_COLOR          1U
#define PROFILE_LINE_ONLY           2U
#define PROFILE_OBSTACLE_ONLY       3U
#define PROFILE_TORQUE_ONLY         4U
#define PROFILE_SEQUENCE_NO_COLOR   5U

#ifndef MISSION_ACTIVE_PROFILE
#define MISSION_ACTIVE_PROFILE PROFILE_FULL_COLOR
#endif

/* Per-profile mode gates */
#if (MISSION_ACTIVE_PROFILE == PROFILE_FULL_COLOR)
#define MISSION_EN_LINE_FOLLOW      1U
#define MISSION_EN_OBSTACLE_AVOID   1U
#define MISSION_EN_TORQUE_RAMP      1U
#define MISSION_EN_ZONE_SWITCH      1U
#define MISSION_EN_SEQUENCE         0U
#elif (MISSION_ACTIVE_PROFILE == PROFILE_LINE_ONLY)
#define MISSION_EN_LINE_FOLLOW      1U
#define MISSION_EN_OBSTACLE_AVOID   0U
#define MISSION_EN_TORQUE_RAMP      0U
#define MISSION_EN_ZONE_SWITCH      0U
#define MISSION_EN_SEQUENCE         0U
#elif (MISSION_ACTIVE_PROFILE == PROFILE_OBSTACLE_ONLY)
#define MISSION_EN_LINE_FOLLOW      0U
#define MISSION_EN_OBSTACLE_AVOID   1U
#define MISSION_EN_TORQUE_RAMP      0U
#define MISSION_EN_ZONE_SWITCH      0U
#define MISSION_EN_SEQUENCE         0U
#elif (MISSION_ACTIVE_PROFILE == PROFILE_TORQUE_ONLY)
#define MISSION_EN_LINE_FOLLOW      0U
#define MISSION_EN_OBSTACLE_AVOID   0U
#define MISSION_EN_TORQUE_RAMP      1U
#define MISSION_EN_ZONE_SWITCH      0U
#define MISSION_EN_SEQUENCE         0U
#else
#define MISSION_EN_LINE_FOLLOW      1U
#define MISSION_EN_OBSTACLE_AVOID   1U
#define MISSION_EN_TORQUE_RAMP      1U
#define MISSION_EN_ZONE_SWITCH      0U
#define MISSION_EN_SEQUENCE         1U
#endif

/* Safety */
#define MISSION_INIT_TIMEOUT_TICKS              120U
#define MISSION_WATCHDOG_NO_PROGRESS_TICKS      300U
#define MISSION_MODE_TIMEOUT_LINE_TICKS        6000U
#define MISSION_MODE_TIMEOUT_OBS_TICKS         2000U
#define MISSION_MODE_TIMEOUT_TORQUE_TICKS      1200U

/* Zone switch filtering */
#define MISSION_ZONE_WINDOW_N          8U
#define MISSION_ZONE_DEBOUNCE_N        5U
#define MISSION_ZONE_LOCKOUT_TICKS     120U
#define MISSION_ZONE_CONF_SWITCH_MIN   700U

#endif
```

---

## 3) Top-Level State Transition Table

| Current Mode | Event / Condition | Next Mode | Notes |
|---|---|---|---|
| `LINE_FOLLOW` | Zone request = GREEN | `LINE_FOLLOW` | No-op if already active |
| `LINE_FOLLOW` | Zone request = YELLOW and profile enables obstacle | `OBSTACLE_AVOID` | Debounce + lockout required |
| `LINE_FOLLOW` | Zone request = BLUE and profile enables torque | `TORQUE_RAMP` | Debounce + lockout required |
| `LINE_FOLLOW` | Zone request = RED | `STOP` | Immediate safe stop |
| `LINE_FOLLOW` | Mode timeout / watchdog trip | `STOP` | Safety |
| `OBSTACLE_AVOID` | Zone request = GREEN and line mode enabled | `LINE_FOLLOW` | Return to tracking |
| `OBSTACLE_AVOID` | Zone request = BLUE and torque enabled | `TORQUE_RAMP` | Profile-gated |
| `OBSTACLE_AVOID` | Zone request = RED | `STOP` | Safety |
| `OBSTACLE_AVOID` | Mode timeout / watchdog trip | `STOP` | Safety |
| `TORQUE_RAMP` | Ramp complete and line mode enabled | `LINE_FOLLOW` | Default handoff |
| `TORQUE_RAMP` | Zone request = RED | `STOP` | Safety |
| `TORQUE_RAMP` | Mode timeout / watchdog trip | `STOP` | Safety |
| `STOP` | Zone request GREEN and profile enables line | `LINE_FOLLOW` | Controlled restart |
| `STOP` | Zone request YELLOW and profile enables obstacle | `OBSTACLE_AVOID` | Controlled restart |
| `STOP` | Zone request BLUE and profile enables torque | `TORQUE_RAMP` | Controlled restart |

