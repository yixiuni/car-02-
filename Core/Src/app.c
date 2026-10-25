#include "app.h"
#include "board_c10b.h"
#include "car_config.h"
#include "encoder.h"
#include "hiwonder_gray.h"
#include "imu_filter.h"
#include "mpu6050.h"
#include "motor_test.h"
#include "gray_learn.h"
#if CAR_TASK1_TEST_BUILD
#include "task1_test.h"
#if CAR_DUAL_MODE_BUILD
#include "course_mode.h"
#include "oled.h"
#endif
#if CAR_CALIBRATION_CONFIRMED
#error "Task 1 test must keep the general course calibration locked"
#endif
volatile uint32_t task1_test_request;
volatile uint32_t task1_test_magic, task1_test_phase, task1_test_trace_count;
#if CAR_DUAL_MODE_BUILD
volatile uint32_t course_run_settings; /* frozen at start: task | laps << 8 */
volatile CourseRunMetrics course_run_metrics;
#endif
typedef struct {
    uint32_t now_ms;
    uint8_t state, fault, point_events, black_mask;
    float x_mm, y_mm, heading_rad;
    int32_t counts_a, counts_b;
    uint16_t left_pwm, right_pwm;
    uint32_t signal_flags;
} Task1Trace;
volatile Task1Trace task1_test_trace[CAR_TEST_TRACE_CAPACITY];
#if CAR_FOUR_TEST_BUILD || CAR_DUAL_MODE_BUILD
_Static_assert(CAR_TEST_TRACE_CAPACITY >=
               (CAR_TEST_RUN_LIMIT_MS+1000U)/CAR_TEST_TRACE_PERIOD_MS+128U,
               "Four-lap trace must retain the full time window and route events");
#endif
static void Task1WatchdogStart(void)
{
    DBGMCU->CR &= ~DBGMCU_CR_DBG_IWDG_STOP;
    IWDG->KR=0xCCCCU; IWDG->KR=0x5555U;
    IWDG->PR=3U; IWDG->RLR=124U;
    while (IWDG->SR!=0U) {}
    IWDG->KR=0xAAAAU;
}
#endif

volatile AppDiagnostics app_diagnostics;
static CarControl car;
static Encoder encoder;
static ImuFilter imu;

void App_Run(void)
{
#if CAR_GRAY_LEARN_BUILD
    GrayLearn_Run();
    return;
#endif
#if CAR_MOTOR_TEST_BUILD
    MotorTest_Run();
    return;
#endif
    Board_Init();
#if CAR_DUAL_MODE_BUILD
    Oled_Init();
    uint32_t display_elapsed=0;
#endif
    uint32_t now=HAL_GetTick();
    HiwonderGray_Init(now);
    CarGeometry geometry=CarGeometry_Default();
#if CAR_TASK1_TEST_BUILD
    geometry.confirmed=true; /* Only this isolated one-shot test image. */
#endif
    Encoder_Init(&encoder,now); ImuFilter_Init(&imu);
    uint8_t identity=0;
    bool mpu_configured=MPU6050_Init(&identity);
    uint32_t mpu_retry_at=now, mpu_wake_at=HAL_GetTick();
    CarControl_Init(&car,&geometry,HAL_GetTick(),Board_ButtonPressed());
#if CAR_TASK1_TEST_BUILD
    car.task=CAR_FOUR_TEST_BUILD ? 4 : CAR_CROSS_TEST_BUILD ? 3 : (CAR_ARC_TEST_BUILD || CAR_LAP_TEST_BUILD) ? 2 : 1;
    Task1Test gate={0};
#if CAR_DUAL_MODE_BUILD
    CourseMode mode;
    CourseMode_Init(&mode,&car);
#endif
    Task1WatchdogStart();
    task1_test_magic=CAR_TIME_TRIAL_BUILD ? 0x54333031U : CAR_DUAL_MODE_BUILD ? 0x44554C35U : CAR_FOUR_TEST_BUILD ? 0x54534B37U : CAR_CROSS_TEST_BUILD ? 0x43525333U : CAR_LAP_TEST_BUILD ? 0x4C415031U : CAR_ARC_TEST_BUILD ? 0x41524331U : 0x54415331U;
    uint32_t fed_at=HAL_GetTick(), trace_at=0, trace_run_at=0;
    uint32_t traced_state=CAR_STOPPED, traced_points=0, traced_signals=0;
    bool tracing=false;
#endif
    uint32_t sampled_at=HAL_GetTick(), point_events=0, mode_events=0, signal_at=0;
    uint8_t mode_flashes=0;
    bool point_signal=false;
    bool brake_applied=false;
    while (1) {
#if CAR_TASK1_TEST_BUILD
        DBGMCU->CR &= ~DBGMCU_CR_DBG_IWDG_STOP;
#endif
        now=HAL_GetTick(); HiwonderGray_Poll(now);
        bool enabled=Board_MotionEnabled();
        if (!enabled) { Board_MotorApply(0,0);brake_applied=false; }
        if (!mpu_configured && !CarControl_Active(&car) && now-mpu_retry_at>=1000) {
            mpu_retry_at=now; mpu_configured=MPU6050_Init(&identity);
            mpu_wake_at=HAL_GetTick();
        }
        if (now-sampled_at>=CAR_CONTROL_PERIOD_MS) {
            float z=0;
            bool sample_ok=mpu_configured && now-mpu_wake_at>=100 && MPU6050_ReadGyroZ(&z);
            if (mpu_configured && now-mpu_wake_at>=100 && !sample_ok) mpu_configured=false;
            now=HAL_GetTick();
            uint32_t interval=now-sampled_at; sampled_at=now;
            int64_t old_a=encoder.counts_a, old_b=encoder.counts_b;
            Encoder_Update(&encoder,&geometry,now);
            bool still=encoder.counts_a==old_a && encoder.counts_b==old_b && !CarControl_Active(&car);
            ImuFilter_Update(&imu,now,z,sample_ok,still);
            GrayProtocol gray; HiwonderGray_Snapshot(&gray);
            CarInputs in={.now_ms=now,.gray_samples=gray.samples,
                .pressed=Board_ButtonPressed(),.enabled=enabled,
                .gray_online=GrayProtocol_Online(&gray,now),.imu_ready=imu.ready && imu.online,
                .encoder_valid=encoder.valid,.black_mask=GrayProtocol_BlackMask(gray.raw_status),
                .yaw_rad=imu.yaw_rad,.left_mm=encoder.left_mm,.right_mm=encoder.right_mm,
                .left_speed=encoder.left_speed,.right_speed=encoder.right_speed};
#if CAR_TASK1_TEST_BUILD
            uint32_t request=task1_test_request;
            if (request) task1_test_request=0;
#if CAR_DUAL_MODE_BUILD
            /* This image is operated by physical gestures only. */
            CourseMode_Apply(&mode,&gate,&in,&car);
            if(gate.recalibrate) {
                display_elapsed=0;
                course_run_metrics=(CourseRunMetrics){0};
                course_run_settings=(uint32_t)car.task | ((uint32_t)car.target_laps<<8);
                task1_test_trace_count=0;tracing=false;
                trace_at=trace_run_at=0;
                traced_state=CAR_STOPPED;traced_points=traced_signals=0;
                point_events=0;point_signal=false;mode_flashes=0;
            }
#else
            Task1Test_Apply(&gate,&in,car.state,request);
#endif
            if (gate.recalibrate) { ImuFilter_Init(&imu); in.imu_ready=false; }
            task1_test_phase=gate.phase;
#endif
#if CAR_DUAL_MODE_BUILD
            bool was_active=CarControl_Active(&car);
            if(was_active) display_elapsed=now-car.run_at;
            if(interval>oled_diagnostics.max_control_interval_ms)
                oled_diagnostics.max_control_interval_ms=interval;
#endif
            CarControl_Update(&car,&in);
#if CAR_DUAL_MODE_BUILD
            if(CarControl_Active(&car)) {
                if(car.left_pwm>course_run_metrics.peak_left_pwm)
                    course_run_metrics.peak_left_pwm=car.left_pwm;
                if(car.right_pwm>course_run_metrics.peak_right_pwm)
                    course_run_metrics.peak_right_pwm=car.right_pwm;
                if(car.left_pwm>=CAR_PWM_LIMIT || car.right_pwm>=CAR_PWM_LIMIT)
                    ++course_run_metrics.saturated_updates;
                ++course_run_metrics.active_updates;
                if(interval>course_run_metrics.max_interval_ms)
                    course_run_metrics.max_interval_ms=interval;
            }
#endif
            brake_applied=CarControl_BrakeActive(&car,now) && Board_MotorBrake();
            if(!brake_applied) Board_MotorApply(car.left_pwm,car.right_pwm);
            if (point_events!=car.point_events) {
                point_events=car.point_events; signal_at=now; point_signal=true; mode_flashes=0;
            }
            if (mode_events!=car.mode_events) {
                mode_events=car.mode_events; mode_flashes=car.task; signal_at=now; point_signal=false;
#if CAR_DUAL_MODE_BUILD
                mode_flashes=mode.feedback_pulses;
#endif
            }
            app_diagnostics=(AppDiagnostics){
                .state=car.state,.fault=car.fault,.enabled=enabled,.task=car.task,.segment=car.segment,
                .calibration_valid=CarGeometry_Valid(&geometry),.gray_online=in.gray_online,
                .mpu_online=imu.online,.imu_ready=imu.ready,.encoder_valid=encoder.valid,
                .mpu_identity=identity,.gray_raw_status=gray.raw_status,.black_mask=in.black_mask,
                .completed_laps=car.completed_laps,.target_laps=car.target_laps,
                .left_pwm=car.left_pwm,.right_pwm=car.right_pwm,
                .gray_samples=gray.samples,.gray_timeouts=gray.timeouts,.gray_errors=gray.errors,
                .gray_unexpected_bytes=gray.unexpected_bytes,.point_events=car.point_events,
                .last_point=car.last_point,.gyro_calibration_samples=imu.count,.control_interval_ms=interval,
                .encoder_a_counts=encoder.counts_a,.encoder_b_counts=encoder.counts_b,
                .line_error_mm=car.line_error,.gyro_z_dps=z,.gyro_bias_dps=imu.bias_dps,.yaw_rad=imu.yaw_rad,
                .x_mm=car.x_mm,.y_mm=car.y_mm,.heading_rad=car.heading_rad,
                .left_mm=encoder.left_mm,.right_mm=encoder.right_mm,
                .left_speed=encoder.left_speed,.right_speed=encoder.right_speed,
                .left_target=car.left_target,.right_target=car.right_target};
        }
        now=HAL_GetTick();
        if (point_signal && now-signal_at>=CAR_SIGNAL_MS) point_signal=false;
        bool mode_signal=false;
        if (mode_flashes) {
            uint32_t phase=(now-signal_at)/CAR_SIGNAL_MS;
            if (phase>=2U*mode_flashes) mode_flashes=0;
            else mode_signal=(phase%2U)==0;
        }
        Board_SetBuzzer(point_signal || mode_signal);
        /* Light immediately at A; pulse off at later moving point events. */
        bool run=CarControl_Active(&car);
        bool start_signal=point_signal && car.last_point==CAR_POINT_A && car.point_events==1;
        bool run_led=(run && (!point_signal || start_signal)) || (!run && point_signal) || mode_signal;
        Board_SetLeds(run_led,
                      car.fault!=CAR_OK || !app_diagnostics.calibration_valid ||
                      !app_diagnostics.gray_online || !app_diagnostics.imu_ready);
#if CAR_TASK1_TEST_BUILD
        if (!tracing && car.point_events) { tracing=true; trace_run_at=now; }
        uint32_t signals=(point_signal || mode_signal ? 1U : 0U) |
            (run_led ? 2U : 0U) | (brake_applied ? 4U : 0U);
        if (tracing && now-trace_run_at<=CAR_TEST_RUN_LIMIT_MS+1000U && task1_test_trace_count<CAR_TEST_TRACE_CAPACITY &&
            (!task1_test_trace_count || now-trace_at>=CAR_TEST_TRACE_PERIOD_MS || traced_state!=(uint32_t)car.state ||
             traced_points!=car.point_events || traced_signals!=signals)) {
            task1_test_trace[task1_test_trace_count]=(Task1Trace){now,
                (uint8_t)car.state,(uint8_t)car.fault,(uint8_t)car.point_events,
                app_diagnostics.black_mask,car.x_mm,car.y_mm,car.heading_rad,
                (int32_t)encoder.counts_a,(int32_t)encoder.counts_b,
                car.left_pwm,car.right_pwm,signals};
            ++task1_test_trace_count; trace_at=now;
            traced_state=car.state; traced_points=car.point_events; traced_signals=signals;
        }
        if (now!=fed_at) { IWDG->KR=0xAAAAU; fed_at=now; }
#endif
#if CAR_DUAL_MODE_BUILD
        OledView view={.task=car.task,.laps=car.target_laps,.completed=car.completed_laps,
            .state=car.state,.fault=car.fault,.phase=gate.phase,.active=run,
            .ready=enabled && app_diagnostics.calibration_valid && app_diagnostics.gray_online &&
                app_diagnostics.imu_ready && app_diagnostics.encoder_valid,
            .elapsed_ms=display_elapsed};
        Oled_Poll(HAL_GetTick(),&view);
#endif
        HAL_Delay(1);
    }
}
