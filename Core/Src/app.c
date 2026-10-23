#include "app.h"
#include "board_c10b.h"
#include "car_config.h"
#include "encoder.h"
#include "hiwonder_gray.h"
#include "imu_filter.h"
#include "mpu6050.h"

volatile AppDiagnostics app_diagnostics;
static CarControl car;
static Encoder encoder;
static ImuFilter imu;

void App_Run(void)
{
    Board_Init();
    uint32_t now=HAL_GetTick();
    HiwonderGray_Init(now);
    CarGeometry geometry=CarGeometry_Default();
    Encoder_Init(&encoder,now); ImuFilter_Init(&imu);
    uint8_t identity=0;
    bool mpu_configured=MPU6050_Init(&identity);
    uint32_t mpu_retry_at=now, mpu_wake_at=HAL_GetTick();
    CarControl_Init(&car,&geometry,HAL_GetTick(),Board_ButtonPressed());
    uint32_t sampled_at=HAL_GetTick(), point_events=0, mode_events=0, signal_at=0;
    uint8_t mode_flashes=0;
    bool point_signal=false;
    while (1) {
        now=HAL_GetTick(); HiwonderGray_Poll(now);
        bool enabled=Board_MotionEnabled();
        if (!enabled) Board_MotorApply(0,0);
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
            CarControl_Update(&car,&in); Board_MotorApply(car.left_pwm,car.right_pwm);
            if (point_events!=car.point_events) {
                point_events=car.point_events; signal_at=now; point_signal=true; mode_flashes=0;
            }
            if (mode_events!=car.mode_events) {
                mode_events=car.mode_events; mode_flashes=car.task; signal_at=now; point_signal=false;
            }
            app_diagnostics=(AppDiagnostics){
                .state=car.state,.fault=car.fault,.enabled=enabled,.task=car.task,.segment=car.segment,
                .calibration_valid=CarGeometry_Valid(&geometry),.gray_online=in.gray_online,
                .mpu_online=imu.online,.imu_ready=imu.ready,.encoder_valid=encoder.valid,
                .mpu_identity=identity,.gray_raw_status=gray.raw_status,.black_mask=in.black_mask,
                .completed_laps=car.completed_laps,.left_pwm=car.left_pwm,.right_pwm=car.right_pwm,
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
        /* RUN LED pulses off at point events while moving; on at final stop. */
        bool run=CarControl_Active(&car);
        Board_SetLeds((run && !point_signal) || (!run && point_signal) || mode_signal,
                      car.fault!=CAR_OK || !app_diagnostics.calibration_valid ||
                      !app_diagnostics.gray_online || !app_diagnostics.imu_ready);
        HAL_Delay(1);
    }
}
