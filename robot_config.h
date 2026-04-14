#ifndef ROBOT_CONFIG_H_
#define ROBOT_CONFIG_H_

/*
 * Motor tuning
 */
#define ROBOT_CFG_MOTOR_PWM_PERIOD_TICKS           375U

/*
 * Line-follow tuning
 */
#define ROBOT_CFG_LINE_FOLLOW_SAMPLE_US            1000U
#define ROBOT_CFG_LINE_FOLLOW_RESOLVE_SAMPLE_US    2000U
#define ROBOT_CFG_LINE_MASK_LEFT                   0xE0U
#define ROBOT_CFG_LINE_MASK_CENTER                 0x18U
#define ROBOT_CFG_LINE_MASK_RIGHT                  0x07U
#define ROBOT_CFG_LINE_SPEED_FORWARD_LEFT          170U
#define ROBOT_CFG_LINE_SPEED_FORWARD_RIGHT         170U
#define ROBOT_CFG_LINE_SPEED_TURN_LEFT             130U
#define ROBOT_CFG_LINE_SPEED_TURN_RIGHT            130U
#define ROBOT_CFG_LINE_DUTY_MIN                    60U
#define ROBOT_CFG_LINE_DUTY_SLOW                   110U
#define ROBOT_CFG_LINE_DUTY_CRUISE                 170U
#define ROBOT_CFG_LINE_DUTY_FAST                   210U
#define ROBOT_CFG_LINE_DUTY_PIVOT                  120U
#define ROBOT_CFG_LINE_E_CENTER                    300
#define ROBOT_CFG_LINE_E_SHARP                     1400
#define ROBOT_CFG_LINE_E_RECOVER_EXIT              500
#define ROBOT_CFG_LINE_KP_NUM                      1
#define ROBOT_CFG_LINE_KD_NUM                      4
#define ROBOT_CFG_LINE_ERR_DIV                     100
#define ROBOT_CFG_LINE_CORR_CLAMP_NORMAL           90
#define ROBOT_CFG_LINE_CORR_CLAMP_SHARP            150
#define ROBOT_CFG_LINE_N_CENTER                    3U
#define ROBOT_CFG_LINE_GENTLE_N                    2U
#define ROBOT_CFG_LINE_SHARP_N                     2U
#define ROBOT_CFG_LINE_SAT_CYCLES                  50U
#define ROBOT_CFG_LINE_SWEEP_CYCLES                100U
#define ROBOT_CFG_LINE_RECOVER_REVERSE_CYCLES      15U
#define ROBOT_CFG_LINE_RECOVER_REACQUIRE_N         2U
#define ROBOT_CFG_LINE_RECOVER_VERIFY_CYCLES       20U
#define ROBOT_CFG_LINE_RECOVER_MAX_CYCLES          ((2U * ROBOT_CFG_LINE_SWEEP_CYCLES) + \
                                                    ROBOT_CFG_LINE_RECOVER_REVERSE_CYCLES + \
                                                    ROBOT_CFG_LINE_RECOVER_VERIFY_CYCLES)
#define ROBOT_CFG_LINE_WIDE_CYCLES                 90U
#define ROBOT_CFG_LINE_GAP_N                       2U
#define ROBOT_CFG_LINE_GAP_MAX_CYCLES              25U
#define ROBOT_CFG_LINE_INTERSECT_COUNT_THR         6U
#define ROBOT_CFG_LINE_INTERSECT_N                 2U
#define ROBOT_CFG_LINE_INTERSECT_LOCK_CYCLES       35U
#define ROBOT_CFG_LINE_OBS_WARN_CM                 12.0f

/*
 * PID line-follow feature control
 */
#ifndef ENABLE_STARTUP_LINE_ACQUIRE_FIX
#define ENABLE_STARTUP_LINE_ACQUIRE_FIX         1U
#endif
#ifndef ENABLE_PID_SOFT_START_FIX
#define ENABLE_PID_SOFT_START_FIX               1U
#endif
#ifndef ENABLE_LINE_DEBUG_WATCH
#define ENABLE_LINE_DEBUG_WATCH                 1U
#endif
#ifndef ENABLE_UART_TUNING_DEBUG
#define ENABLE_UART_TUNING_DEBUG                1U
#endif
#ifndef ENABLE_PID_LINE_FOLLOW
#define ENABLE_PID_LINE_FOLLOW                  1U
#endif
#ifndef ENABLE_PID_INTEGRAL
#define ENABLE_PID_INTEGRAL                     1U
#endif
#ifndef ENABLE_PID_DEBUG_WATCH
#define ENABLE_PID_DEBUG_WATCH                  1U
#endif

/*
 * PID line-follow tuning
 *
 * Tuning order:
 * 1. Set KI_LINE = 0 and KD_LINE = 0, then raise KP_LINE slowly.
 *    If steering reacts too weakly, KP_LINE is too low.
 *    If the robot turns too much or oscillates, KP_LINE is too high.
 * 2. After KP is reasonable, raise KD_LINE slowly.
 *    KD_LINE reduces wobble and overshoot.
 *    If steering becomes noisy or jerky, KD_LINE is too high.
 * 3. Add only a small KI_LINE after KP and KD are stable.
 *    KI_LINE helps correct steady bias to one side.
 *    If correction builds up over time or becomes unstable, KI_LINE is too high.
 *
 * Safety:
 * - tune at low speed first
 * - keep PID_CORR_MAX bounded
 * - keep PID_I_SUM_MAX bounded
 * - reset integral when the line is lost
 * - lower PID_BASE_DUTY first if the robot is still too aggressive
 */
#define KP_LINE                                    1
#define KI_LINE                                    0
#define KD_LINE                                    1
#define PID_SCALE                                100
#define PID_CORR_MAX                              60
#define PID_I_SUM_MAX                            600
#define PID_D_TERM_MAX                            60
#define PID_DEADBAND_ERR                         120
#define PID_BASE_DUTY                            130U
#define ROBOT_CFG_LINE_START_BOOST_DUTY          150U
#define STARTUP_ACQUIRE_N                          2U
#define STARTUP_SETTLE_MS                         50U
#define UART_DEBUG_EVERY_N_TICKS                 25U
#define ROBOT_CFG_LINE_FULL_WIDTH_N               7U
#define ROBOT_CFG_LINE_LOST_CONFIRM_N             2U
#define ROBOT_CFG_LINE_CORNER_ASSIST_CYCLES      12U
#define ROBOT_CFG_LINE_CORNER_PIVOT_DUTY         125U

/*
 * System mode selection
 */
#define SYSTEM_MODE_DEBUG                       1U
#define SYSTEM_MODE_REAL                        2U

#ifndef ROBOT_SYSTEM_MODE
#define ROBOT_SYSTEM_MODE                       SYSTEM_MODE_DEBUG
#endif

#if (ROBOT_SYSTEM_MODE == SYSTEM_MODE_DEBUG)
#define ENABLE_UART_DEBUG                       1U
#define ENABLE_SENSOR_PRINTS                    1U
#define ENABLE_STATE_PRINTS                     1U
#define ENABLE_CASE_PRINTS                      1U
#define ROBOT_CFG_DEBUG_PRINT_DIVIDER           20U
#else
#define ENABLE_UART_DEBUG                       0U
#define ENABLE_SENSOR_PRINTS                    0U
#define ENABLE_STATE_PRINTS                     0U
#define ENABLE_CASE_PRINTS                      0U
#define ROBOT_CFG_DEBUG_PRINT_DIVIDER           0U
#endif

/*
 * Ramp-follow tuning
 */
#ifndef ENABLE_OBSTACLE_FEATURE
#define ENABLE_OBSTACLE_FEATURE                  1U
#endif
#ifndef ENABLE_RAMP_FEATURE
#define ENABLE_RAMP_FEATURE                      0U
#endif
#ifndef ENABLE_PI_AI_HINT_FEATURE
#define ENABLE_PI_AI_HINT_FEATURE                0U
#endif
#ifndef ENABLE_BOUNDARY_FOLLOW_FEATURE
#define ENABLE_BOUNDARY_FOLLOW_FEATURE           0U
#endif
#ifndef ENABLE_REACQUIRE_GATING_FEATURE
#define ENABLE_REACQUIRE_GATING_FEATURE          0U
#endif
#ifndef ENABLE_IMU_HEADING_GATE
#define ENABLE_IMU_HEADING_GATE                  0U
#endif
#ifndef ROBOT_CFG_ENABLE_RAMP_FEATURE
#define ROBOT_CFG_ENABLE_RAMP_FEATURE             ENABLE_RAMP_FEATURE
#endif
#define ROBOT_CFG_RAMP_PITCH_ENTER_DEG             8
#define ROBOT_CFG_RAMP_PITCH_EXIT_DEG              4
#define ROBOT_CFG_RAMP_N                           3U
#define ROBOT_CFG_RAMP_EXIT_N                      4U
#define ROBOT_CFG_RAMP_FACTOR_PCT                  70U
#define ROBOT_CFG_RAMP_DOWN_FACTOR_PCT             60U
#define ROBOT_CFG_RAMP_US_SYMM_CM                  8U
#define ROBOT_CFG_RAMP_DOWN_CORR_CLAMP             60
#define ROBOT_CFG_AVOID_OBS_CONFIRM_N              3U
#define ROBOT_CFG_AVOID_IR_CONFIRM_RAW             9000U

/*
 * Obstacle tuning
 */
#define ROBOT_CFG_OBS_WARN_CM                    20U
#define ROBOT_CFG_OBS_STOP_CM                    12U
#define ROBOT_CFG_IR_WARN_MM                     180U
#define ROBOT_CFG_IR_STOP_MM                     120U
#define ROBOT_CFG_OBS_CONF_THR                   5U
#define ROBOT_CFG_OBS_CONFIRM_N                  2U
#define ROBOT_CFG_DUTY_BACKOFF                   ROBOT_CFG_LINE_DUTY_SLOW
#define ROBOT_CFG_BACKOFF_MS                     250U
#define ROBOT_CFG_BACKOFF_MS_MAX                 600U
#define ROBOT_CFG_BACKOFF_STEP_MS                100U
#define ROBOT_CFG_BYPASS_SETTLE_MS               150U
#define ROBOT_CFG_PIVOT_MS                       300U
#define ROBOT_CFG_PIVOT_MS_2                     300U
#define ROBOT_CFG_CLEAR_MS                       600U
#define ROBOT_CFG_D_SIDE_CM                      15
#define ROBOT_CFG_KP_SIDE                        6
#define ROBOT_CFG_KD_SIDE                        3
#define ROBOT_CFG_AVOID_TIMEOUT_MS               4000U
#define ROBOT_CFG_SIDE_CORR_MAX                  70
#define ROBOT_CFG_FRONT_BLOCK_EXTRA_TURN         25
#define ROBOT_CFG_SIDE_SELECT_DIFF_CM            4U
#define ROBOT_CFG_OPEN_SIDE_MARGIN_CM            5U
#define ROBOT_CFG_REACQUIRE_N                    3U
#define ROBOT_CFG_MIN_BYPASS_FORWARD_MS          250U
#define ROBOT_CFG_YAW_ACCEPT_DEG                 90
#define ROBOT_CFG_REACQUIRE_ACCEPT_N             3U
#define ROBOT_CFG_REACQUIRE_E_ACCEPT             900
#define ROBOT_CFG_REACQUIRE_SWEEP_STEP_MS        60U
#define ROBOT_CFG_REACQUIRE_FORWARD_STEP_MS      80U
#define ROBOT_CFG_REACQUIRE_PRIMARY_TIMEOUT_MS   1500U
#define ROBOT_CFG_REACQUIRE_SECONDARY_TIMEOUT_MS 1500U
#define ROBOT_CFG_OBS_CLEAR_CM                   40U
#define ROBOT_CFG_IR_CLEAR_MM                    220U
#define ROBOT_CFG_MAX_CLEAR_EXTEND_MS            400U
#define ROBOT_CFG_AI_DATA_MAX_AGE_TICKS          3U
#define ROBOT_CFG_OBS_CONF_MAX                   20U
#define ROBOT_CFG_AVOID_NEAR_DELTA_CM              3.0f
#define ROBOT_CFG_AVOID_FAR_DELTA_CM               4.0f
#define ROBOT_CFG_AVOID_INVALID_CM                 0.0f
#define ROBOT_CFG_AVOID_TURN_LEFT_DUTY            120U
#define ROBOT_CFG_AVOID_TURN_RIGHT_DUTY           120U

/*
 * Ultrasonic timing tuning
 */
#define ROBOT_CFG_ULTRA_TRIGGER_PULSE_US          10U
#define ROBOT_CFG_ULTRA_START_TIMEOUT_US          30000U
#define ROBOT_CFG_ULTRA_ECHO_TIMEOUT_US           30000U
#define ROBOT_CFG_ULTRA_INVALID_CM                (-1.0f)

/*
 * Calibration tuning
 */
#define ROBOT_CFG_CAL_LINE_SAMPLE_US              1000U
#define ROBOT_CFG_CAL_LINE_SAMPLE_COUNT           32U
#define ROBOT_CFG_CAL_BUMP_VERIFY_MS              2000U
#define ROBOT_CFG_CAL_STAGE_PREP_MS               1500U
#define ROBOT_CFG_CAL_STAGE_DONE_MS               2000U
#define ROBOT_CFG_CAL_ULTRA_SAMPLE_COUNT          8U
#define ROBOT_CFG_CAL_ULTRA_INTER_SAMPLE_MS       60U

/*
 * Emergency tuning
 */
#define ROBOT_CFG_EMERGENCY_LATCH_UNTIL_RESET     1U

/*
 * Demo mode tuning
 *
 * Demo timing is expressed in main-loop update cycles. With the current
 * ROBOT_CFG_MAIN_CONTROL_LOOP_MS value, these counts map directly to a
 * staged integrated verification flow.
 */
#define ROBOT_CFG_DEMO_STARTUP_CYCLES             30U
#define ROBOT_CFG_DEMO_SENSOR_STAGE_CYCLES        80U
#define ROBOT_CFG_DEMO_LINE_STAGE_CYCLES          120U
#define ROBOT_CFG_DEMO_AVOID_STAGE_CYCLES         120U
#define ROBOT_CFG_DEMO_COLLISION_STAGE_CYCLES     120U

/*
 * App/debug tuning
 */
#define ROBOT_MODE_NORMAL                         0U
#define ROBOT_MODE_DEMO                           1U
#define ROBOT_CFG_APP_MODE_CALIBRATION            0U
#define ROBOT_CFG_APP_MODE                        ROBOT_MODE_NORMAL
#define ROBOT_CFG_RUN_STARTUP_CALIBRATION         0U
#define ROBOT_CFG_MAIN_CONTROL_LOOP_MS            10U
#define ROBOT_CFG_MAIN_CAL_WATCH_UPDATE_MS        100U

#ifndef ROBOT_CFG_IR_BUSY_TIMEOUT_LOOPS
#define ROBOT_CFG_IR_BUSY_TIMEOUT_LOOPS       60000U
#endif

/*
 * MPU6500 tuning
 */
#ifndef ENABLE_MPU6500_FEATURE
#define ENABLE_MPU6500_FEATURE                1U
#endif
#ifndef ENABLE_MPU6500_CALIBRATION
#define ENABLE_MPU6500_CALIBRATION            0U
#endif
#ifndef ENABLE_MPU6500_MOTION_DIAG
#define ENABLE_MPU6500_MOTION_DIAG            0U
#endif
#define MPU6500_I2C_SLAVE_ADDR                0x68U
#define MPU6500_SAMPLE_DIV                    9U
#define MPU6500_DLPF_CFG                      3U
#define MPU6500_GYRO_FS_SEL                   0U
#define MPU6500_ACCEL_FS_SEL                  0U
#define MPU6500_PITCH_FILTER_ALPHA            95U
#define MPU6500_CALIB_SAMPLES                 128U
#define MPU6500_STILL_GYRO_THR                200U
#define MPU6500_STILL_ACCEL_THR               2000U

#endif /* ROBOT_CONFIG_H_ */
