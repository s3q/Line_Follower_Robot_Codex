#ifndef MISSION_CONFIG_H_
#define MISSION_CONFIG_H_

#include "robot_config.h"

/*
 * Compile-time mission profiles.
 */
#define PROFILE_FULL_COLOR          1U
#define PROFILE_LINE_ONLY           2U
#define PROFILE_OBSTACLE_ONLY       3U
#define PROFILE_TORQUE_ONLY         4U
#define PROFILE_SEQUENCE_NO_COLOR   5U

#ifndef MISSION_ACTIVE_PROFILE
#define MISSION_ACTIVE_PROFILE PROFILE_FULL_COLOR
#endif

/*
 * 1 = switch modes only from zone detector output.
 * 0 = fallback switching (pitch->torque, obstacle->avoid, else line follow).
 */
#ifndef USE_COLOR_ZONES
#define USE_COLOR_ZONES                     1U
#endif

/*
 * Mode feature gating per profile.
 */
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
#else /* PROFILE_SEQUENCE_NO_COLOR */
#define MISSION_EN_LINE_FOLLOW      1U
#define MISSION_EN_OBSTACLE_AVOID   1U
#define MISSION_EN_TORQUE_RAMP      1U
#define MISSION_EN_ZONE_SWITCH      0U
#define MISSION_EN_SEQUENCE         1U
#endif

/*
 * Mission watchdog / timeout pattern.
 * All time-based limits are in control-loop ticks.
 */
#define MISSION_INIT_TIMEOUT_TICKS              120U
#define MISSION_MODE_TIMEOUT_LINE_TICKS        6000U
#define MISSION_MODE_TIMEOUT_OBS_TICKS         2000U
#define MISSION_MODE_TIMEOUT_TORQUE_TICKS      1200U
#define MISSION_WATCHDOG_NO_PROGRESS_TICKS      300U

/*
 * Conservative switching limits.
 */
#define MISSION_MODE_SWITCH_LOCKOUT_TICKS        80U
#define MISSION_MODE_MIN_DWELL_TICKS             20U

/*
 * Sequence enforcement:
 * green(line) -> yellow(obstacle) -> blue(torque) -> red(stop).
 */
#define MISSION_SEQUENCE_ENFORCE                 1U

/*
 * Zone detector input and filtering.
 * Uses only line reflectance mask from P7.0..P7.7.
 */
#define MISSION_ZONE_SAMPLE_US         ROBOT_CFG_LINE_FOLLOW_SAMPLE_US
#define MISSION_ZONE_WINDOW_N          8U
#define MISSION_ZONE_DEBOUNCE_N        5U
#define MISSION_ZONE_LOCKOUT_TICKS     120U

/*
 * Zone confidence and ambiguity gating.
 * Never switch on low confidence.
 */
#define MISSION_ZONE_CONF_REPORT_MIN   450U
#define MISSION_ZONE_CONF_SWITCH_MIN   700U
#define MISSION_ZONE_SCORE_MAX_ACCEPT   90U
#define MISSION_ZONE_SCORE_MARGIN_MIN    8U
#define MISSION_ZONE_SCORE_MARGIN_TGT   24U

/*
 * Feature hard gates.
 */
#define MISSION_ZONE_MEAN_MIN           0
#define MISSION_ZONE_MEAN_MAX           8
#define MISSION_ZONE_SPREAD_MAX         8
#define MISSION_ZONE_MAD_MAX            8

/*
 * Feature weights for score.
 */
#define MISSION_ZONE_W_MEAN            10
#define MISSION_ZONE_W_SPREAD           8
#define MISSION_ZONE_W_MAD              6
#define MISSION_ZONE_W_LR               2
#define MISSION_ZONE_W_CENTER_OUTER     2

/*
 * Zone centroids (tune from logs).
 * Features are based on per-channel hit counts over WINDOW_N samples.
 */
#define ZONE_GREEN_MEAN                7
#define ZONE_GREEN_SPREAD              1
#define ZONE_GREEN_MAD                 1
#define ZONE_GREEN_LR                  0
#define ZONE_GREEN_CO                  0

#define ZONE_YELLOW_MEAN               5
#define ZONE_YELLOW_SPREAD             3
#define ZONE_YELLOW_MAD                2
#define ZONE_YELLOW_LR                 1
#define ZONE_YELLOW_CO                 2

#define ZONE_BLUE_MEAN                 3
#define ZONE_BLUE_SPREAD               2
#define ZONE_BLUE_MAD                  2
#define ZONE_BLUE_LR                   1
#define ZONE_BLUE_CO                   1

#define ZONE_RED_MEAN                  1
#define ZONE_RED_SPREAD                1
#define ZONE_RED_MAD                   1
#define ZONE_RED_LR                    0
#define ZONE_RED_CO                    0

/*
 * Torque ramp mode settings.
 */
#define MISSION_TORQUE_START_DUTY      90U
#define MISSION_TORQUE_TARGET_DUTY    160U
#define MISSION_TORQUE_STEP_DUTY        5U
#define MISSION_TORQUE_STEP_TICKS       4U
#define MISSION_TORQUE_HOLD_TICKS      40U

/*
 * IMU-based torque/ramp mode tuning.
 * Uses complementary-filter outputs:
 * - pitch: gLineFollowPitchInputDeg
 * - yaw:   gAvoidYawInputDeg (when gAvoidYawInputFresh != 0)
 */
#define MISSION_TORQUE_PITCH_ENTER_DEG          ROBOT_CFG_RAMP_PITCH_ENTER_DEG
#define MISSION_TORQUE_PITCH_EXIT_DEG           ROBOT_CFG_RAMP_PITCH_EXIT_DEG
#define MISSION_TORQUE_PITCH_ABORT_DEG          28
#define MISSION_TORQUE_ENTER_CONFIRM_TICKS      3U
#define MISSION_TORQUE_EXIT_CONFIRM_TICKS       4U
#define MISSION_TORQUE_LINE_CONFIRM_TICKS       3U
#define MISSION_TORQUE_LINE_COUNT_MIN           2U
#define MISSION_TORQUE_CLIMB_DUTY               170U
#define MISSION_TORQUE_CLIMB_STEEP_DUTY         190U
#define MISSION_TORQUE_DESCENT_DUTY              95U
#define MISSION_TORQUE_DESCENT_BRAKE_DUTY        75U
#define MISSION_TORQUE_EXIT_CRUISE_DUTY         115U
#define MISSION_TORQUE_YAW_KP_NUM                1
#define MISSION_TORQUE_YAW_KP_DIV                3
#define MISSION_TORQUE_YAW_CORR_MAX             18
#define MISSION_TORQUE_MAX_CLIMB_TICKS         500U
#define MISSION_TORQUE_MAX_DESCENT_TICKS       500U
#define MISSION_TORQUE_EXIT_SEARCH_TICKS       600U

/*
 * Fallback mode switching confirms.
 */
#define MISSION_FALLBACK_OBS_CONFIRM_TICKS        2U
#define MISSION_FALLBACK_TORQUE_CONFIRM_TICKS     3U

#endif /* MISSION_CONFIG_H_ */
