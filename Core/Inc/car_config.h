#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/* Separate opt-in bench firmware; never enabled by a normal build. */
#ifndef CAR_MOTOR_TEST_BUILD
#define CAR_MOTOR_TEST_BUILD 0
#endif
#ifndef CAR_GROUND_TEST_BUILD
#define CAR_GROUND_TEST_BUILD 0
#endif
#ifndef CAR_TASK1_TEST_BUILD
#define CAR_TASK1_TEST_BUILD 0
#endif
#ifndef CAR_GRAY_LEARN_BUILD
#define CAR_GRAY_LEARN_BUILD 0
#endif
#ifndef CAR_ARC_TEST_BUILD
#define CAR_ARC_TEST_BUILD 0
#endif
#ifndef CAR_LAP_TEST_BUILD
#define CAR_LAP_TEST_BUILD 0
#endif
#ifndef CAR_CROSS_TEST_BUILD
#define CAR_CROSS_TEST_BUILD 0
#endif
#ifndef CAR_DUAL_MODE_BUILD
#define CAR_DUAL_MODE_BUILD 0
#endif
#ifndef CAR_FOUR_TEST_BUILD
#define CAR_FOUR_TEST_BUILD 0
#endif
/* Experimental speed planner; the validated DUL5 remains the default. */
#ifndef CAR_TIME_TRIAL_BUILD
#define CAR_TIME_TRIAL_BUILD 0
#endif
#ifndef CAR_TIME_TRIAL_STAGE2
#define CAR_TIME_TRIAL_STAGE2 0
#endif
#ifndef CAR_TIME_TRIAL_STAGE3
#define CAR_TIME_TRIAL_STAGE3 0
#endif
#if CAR_TIME_TRIAL_STAGE2 && CAR_TIME_TRIAL_STAGE3
#error "Choose one time-trial stage"
#endif
#if (CAR_TIME_TRIAL_STAGE2 || CAR_TIME_TRIAL_STAGE3) && !CAR_TIME_TRIAL_BUILD
#error "Stage 2/3 requires the time-trial build"
#endif
#if CAR_TIME_TRIAL_BUILD && !CAR_DUAL_MODE_BUILD
#error "Time trial requires the dual-mode course build"
#endif
#ifndef CAR_TT_STRAIGHT_SPEED
#define CAR_TT_STRAIGHT_SPEED (CAR_TIME_TRIAL_STAGE3 ? 550.0f : CAR_TIME_TRIAL_STAGE2 ? 500.0f : 450.0f)
#endif
#ifndef CAR_TT_ARC_SPEED
#define CAR_TT_ARC_SPEED (CAR_TIME_TRIAL_STAGE3 ? 470.0f : CAR_TIME_TRIAL_STAGE2 ? 430.0f : 370.0f)
#endif
#ifndef CAR_TT_ENTRY_SPEED
#define CAR_TT_ENTRY_SPEED 370.0f
#endif
#ifndef CAR_TT_WHEEL_SPEED
#define CAR_TT_WHEEL_SPEED (CAR_TIME_TRIAL_STAGE3 ? 700.0f : CAR_TIME_TRIAL_STAGE2 ? 650.0f : 620.0f)
#endif
#ifndef CAR_TT_D_FILTER_MS
#define CAR_TT_D_FILTER_MS 10.0f
#endif
#define CAR_TT_APPROACH_ACCEL 1000.0f
#if (CAR_ARC_TEST_BUILD + CAR_LAP_TEST_BUILD + CAR_CROSS_TEST_BUILD + CAR_FOUR_TEST_BUILD + CAR_DUAL_MODE_BUILD) > 1
#error "Choose arc or lap diagnostic"
#endif
#if (CAR_ARC_TEST_BUILD || CAR_LAP_TEST_BUILD || CAR_CROSS_TEST_BUILD || CAR_FOUR_TEST_BUILD || CAR_DUAL_MODE_BUILD) && !CAR_TASK1_TEST_BUILD
#error "Arc test requires the isolated one-shot test gate"
#endif
#define CAR_TEST_RUN_LIMIT_MS ((CAR_FOUR_TEST_BUILD || CAR_DUAL_MODE_BUILD) ? 160000U : CAR_CROSS_TEST_BUILD ? 40000U : CAR_LAP_TEST_BUILD ? 30000U : CAR_ARC_TEST_BUILD ? 25000U : 15000U)
#define CAR_TEST_TRACE_CAPACITY ((CAR_FOUR_TEST_BUILD || CAR_DUAL_MODE_BUILD) ? 1024U : 256U)
#define CAR_TEST_TRACE_PERIOD_MS ((CAR_FOUR_TEST_BUILD || CAR_DUAL_MODE_BUILD) ? 200U : CAR_CROSS_TEST_BUILD ? 200U : CAR_LAP_TEST_BUILD ? 160U : CAR_ARC_TEST_BUILD ? 120U : 80U)
#if CAR_GRAY_LEARN_BUILD && (CAR_TASK1_TEST_BUILD || CAR_MOTOR_TEST_BUILD)
#error "Gray learning cannot include motion diagnostics"
#endif
#if CAR_TASK1_TEST_BUILD && CAR_MOTOR_TEST_BUILD
#error "Task 1 and motor diagnostic builds are exclusive"
#endif
#if CAR_GROUND_TEST_BUILD && !CAR_MOTOR_TEST_BUILD
#error "Ground diagnostic requires the isolated motor-test image"
#endif
#ifndef CAR_MOTOR_SLOW_DECAY
#define CAR_MOTOR_SLOW_DECAY 1 /* raised-wheel A/B comparison: starts at 35% */
#endif
#if CAR_MOTOR_SLOW_DECAY != 0 && CAR_MOTOR_SLOW_DECAY != 1
#error "Invalid motor decay mode"
#endif

/* Physical calibration. Dimensions supplied by the user (2026-10-23).
 * Counts/directions measured with two manual 10-turn runs per wheel.
 * Counts mean quadrature x4 at the WHEEL shaft,
 * including the gearbox, not encoder pulses at the motor shaft. */
#ifndef CAR_CALIBRATION_CONFIRMED
#define CAR_CALIBRATION_CONFIRMED    0
#endif
/* Nominal tire is 65 mm. Trial effective diameter from powered 1 m run:
 * 65 * reported 1025 / encoder 1001.7848 = 66.5063 mm; rounded to 66.5.
 * Independent powered repeat: reported 100 cm, no visible drift.
 * Course calibration lock stays off pending task-specific validation. */
#define CAR_WHEEL_DIAMETER_MM       66.5f
#define CAR_WHEEL_TRACK_MM           120.0f
#define CAR_ENCODER_A_COUNTS_REV  1467.7f
#define CAR_ENCODER_B_COUNTS_REV  1473.3f
#define CAR_ENCODER_A_SIGN           1
#define CAR_ENCODER_B_SIGN          -1
/* Axle midpoint is the navigation origin. Measure front/rear/half-width of
 * the complete car footprint and the sensor bar's forward offset from it. */
#define CAR_FRONT_MM                 170.0f
#define CAR_REAR_MM                  60.0f
#define CAR_HALF_WIDTH_MM            72.5f
#define CAR_SENSOR_FORWARD_MM        165.0f
#define CAR_GRAY_PITCH_MM           15.0f /* user estimate; refine on hardware */
#define CAR_GYRO_Z_SIGN              1    /* positive = counterclockwise */
#define CAR_DEFAULT_TASK            1U

#define CAR_PWM_MAX              1000U
#ifndef CAR_FOUR_PWM_LIMIT
#define CAR_FOUR_PWM_LIMIT        (CAR_TIME_TRIAL_STAGE3 ? 850U : CAR_TIME_TRIAL_STAGE2 ? 800U : 700U)
#endif
#if CAR_FOUR_TEST_BUILD || CAR_DUAL_MODE_BUILD
#define CAR_PWM_LIMIT             CAR_FOUR_PWM_LIMIT /* cruise and steering headroom */
#elif CAR_MOTOR_TEST_BUILD || CAR_TASK1_TEST_BUILD
#define CAR_PWM_LIMIT             350U /* bounded speed-loop bench tests */
#else
#define CAR_PWM_LIMIT             700U
#endif
#define CAR_PWM_FREQUENCY_HZ     20000U
#define CAR_CONTROL_PERIOD_MS       5U
#define CAR_CONTROL_MAX_GAP_MS     50U
#define CAR_BUTTON_DEBOUNCE_MS     25U
#define CAR_BUTTON_HOLD_MS        800U
#define CAR_SENSOR_COUNT            8U
#define CAR_LINE_LOST_TIMEOUT_MS  180U
#define CAR_CAPTURE_TIMEOUT_MS  1800U
#define CAR_STALL_TIMEOUT_MS     700U
#define CAR_SIGNAL_MS            120U
#define CAR_GYRO_CALIBRATION_SAMPLES 400U
#define CAR_GYRO_CALIBRATION_STD_DPS 0.8f
/* Measured stationary Z reference for this installed sensor. It only centers
 * the stillness gate; each startup still estimates bias from 400 samples.
 * Re-measure after replacing the sensor. Do not tune this while rotating. */
#ifndef CAR_GYRO_Z_BIAS_PRIOR_DPS
#define CAR_GYRO_Z_BIAS_PRIOR_DPS (-23.0f * CAR_GYRO_Z_SIGN)
#endif
#define CAR_GYRO_CALIBRATION_WINDOW_DPS 10.0f

/* Initial gains/limits, all require track tuning. Units: mm, s, rad, permille. */
#define CAR_STRAIGHT_SPEED        (CAR_TASK1_TEST_BUILD ? 150.0f : 250.0f)
#define CAR_DIAGONAL_SPEED        220.0f
#ifndef CAR_FOUR_CRUISE_SPEED
#define CAR_FOUR_CRUISE_SPEED     (CAR_DUAL_MODE_BUILD ? 370.0f : 340.0f)
/* Current OLED dual-mode profile: same center speed through every point.
 * Keep the separate older one-shot diagnostic on its validated 340 profile. */
#endif
#define CAR_ARC_SPEED             ((CAR_ARC_TEST_BUILD || CAR_LAP_TEST_BUILD) ? 150.0f : 180.0f)
#define CAR_BRAKE_HOLD_MS          250U /* bounded electrical brake, then coast */
#define CAR_CAPTURE_SPEED          85.0f
#define CAR_ACCEL_MM_S2           350.0f
#ifndef CAR_FOUR_ACCEL_MM_S2
#define CAR_FOUR_ACCEL_MM_S2      2000.0f /* faster differential steering response */
#endif
#define CAR_SPEED_KP                0.8f
#define CAR_SPEED_KI                2.0f
#define CAR_SPEED_FF                1.0f /* initial raised-wheel fit; verify under load */
#define CAR_HEADING_KP              3.0f
#define CAR_LINE_KP                 0.025f
#define CAR_LINE_KD                 0.001f
#define CAR_ENDPOINT_GATE_MM      220.0f
#define CAR_LINE_CONFIRM_FRAMES      3U
#define CAR_ARC_MIN_PROGRESS        0.65f
#define CAR_ARC_MIN_TURN_RAD        2.45f
#define CAR_ENDPOINT_HEADING_RAD    0.44f
#define CAR_ENDPOINT_CORRECTION_MM 60.0f /* max accepted odometry correction */
#define CAR_CROSS_ENDPOINT_CORRECTION_MM 100.0f /* qualified edge only; measured cross-run drift ~78 mm */
#ifndef CAR_CROSS_D_ENTRY_AIM_MM
#define CAR_CROSS_D_ENTRY_AIM_MM    (CAR_TIME_TRIAL_STAGE3 ? 25.0f : 20.0f) /* along left arc beyond D */
#endif
#define CAR_ENDPOINT_YAW_BLEND      0.5f

#ifndef CAR_GRAY_TARGET_BIT_IS_ONE
#define CAR_GRAY_TARGET_BIT_IS_ONE  0 /* measured: white FF, center black C3 */
#endif
#ifndef CAR_GRAY_REVERSE_ORDER
#define CAR_GRAY_REVERSE_ORDER      0
#endif
#define CAR_GRAY_BAUD               115200U
/* MCU-side thresholds from actual track; sensor internal learning untouched. */
#ifndef CAR_GRAY_USE_ANALOG
#define CAR_GRAY_USE_ANALOG 1
#endif
#define CAR_GRAY_BOOT_DELAY_MS      1000U
#define CAR_GRAY_MODE_DELAY_MS      50U
#define CAR_GRAY_QUERY_PERIOD_MS    10U
#define CAR_GRAY_RESPONSE_TIMEOUT_MS 20U
#define CAR_GRAY_SETTLE_MS          2U
#define CAR_GRAY_STALE_TIMEOUT_MS   100U

#define CAR_LEFT_IS_MOTOR_A          1
#define CAR_MOTOR_A_FORWARD_IN1      1
#define CAR_MOTOR_B_FORWARD_IN1      0 /* positive calibrated B counts */
#define CAR_LEFT_SCALE_PERCENT    100U
#define CAR_RIGHT_SCALE_PERCENT   100U
/* Confirm actual buzzer polarity before enabling sound. PA15 is free after
 * disabling JTAG while retaining SWD in Board_Init. */
#define CAR_BUZZER_ACTIVE_HIGH       1

#if CAR_DEFAULT_TASK < 1 || CAR_DEFAULT_TASK > 4
#error "Task must be 1..4"
#endif
#if CAR_CONTROL_PERIOD_MS == 0 || CAR_CONTROL_MAX_GAP_MS <= CAR_CONTROL_PERIOD_MS
#error "Invalid control timing"
#endif
#if (CAR_ENCODER_A_SIGN != 1 && CAR_ENCODER_A_SIGN != -1) || \
    (CAR_ENCODER_B_SIGN != 1 && CAR_ENCODER_B_SIGN != -1) || \
    (CAR_GYRO_Z_SIGN != 1 && CAR_GYRO_Z_SIGN != -1)
#error "Sensor signs must be +1 or -1"
#endif
#if CAR_PWM_FREQUENCY_HZ == 0 || CAR_PWM_LIMIT > CAR_PWM_MAX
#error "Invalid PWM configuration"
#endif
#if (CAR_GRAY_TARGET_BIT_IS_ONE != 0 && CAR_GRAY_TARGET_BIT_IS_ONE != 1) || \
    (CAR_GRAY_REVERSE_ORDER != 0 && CAR_GRAY_REVERSE_ORDER != 1) || \
    (CAR_LEFT_IS_MOTOR_A != 0 && CAR_LEFT_IS_MOTOR_A != 1) || \
    (CAR_MOTOR_A_FORWARD_IN1 != 0 && CAR_MOTOR_A_FORWARD_IN1 != 1) || \
    (CAR_MOTOR_B_FORWARD_IN1 != 0 && CAR_MOTOR_B_FORWARD_IN1 != 1) || \
    (CAR_CALIBRATION_CONFIRMED != 0 && CAR_CALIBRATION_CONFIRMED != 1)
#error "Boolean board configuration must be 0 or 1"
#endif
#if CAR_GRAY_SETTLE_MS == 0 || CAR_GRAY_RESPONSE_TIMEOUT_MS <= CAR_GRAY_SETTLE_MS || \
    CAR_GRAY_STALE_TIMEOUT_MS <= CAR_GRAY_RESPONSE_TIMEOUT_MS
#error "Invalid grayscale timing"
#endif
#endif
