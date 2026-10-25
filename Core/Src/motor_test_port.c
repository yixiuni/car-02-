#include "car_config.h"
#if CAR_MOTOR_TEST_BUILD
#include "motor_test.h"
#include "board_c10b.h"
#include "encoder.h"
#include "ground_test.h"

#if CAR_CALIBRATION_CONFIRMED
#error "Motor test must keep normal course calibration locked"
#endif

/* SWD writes request only; all diagnostic fields are firmware-owned. */
volatile uint32_t motor_test_request;
volatile struct {
    uint32_t magic, now_ms, state, wheel, started_at, reset_flags;
    uint32_t enabled, button_pressed, left_pwm, right_pwm;
    int64_t encoder_a_counts, encoder_b_counts;
} motor_test_diagnostics;

typedef struct {
    uint32_t now_ms;
    int32_t counts_a, counts_b;
    uint16_t left_pwm, right_pwm;
} MotorTestTrace;
volatile MotorTestTrace motor_test_trace[256];
volatile uint32_t motor_test_trace_count;
#if CAR_GROUND_TEST_BUILD
volatile uint32_t ground_test_fault;
#endif

static void WatchdogStart(void)
{
    /* Keep watchdog running even if a debugger halts the core. Nominal
     * timeout 100 ms: LSI /32, reload 124. Do not freeze it via SWD. */
    DBGMCU->CR &= ~DBGMCU_CR_DBG_IWDG_STOP;
    IWDG->KR=0xCCCCU;
    IWDG->KR=0x5555U;
    IWDG->PR=3U;
    IWDG->RLR=124U;
    while (IWDG->SR!=0U) { /* Motors are off; failure causes reset. */ }
    IWDG->KR=0xAAAAU;
}

void MotorTest_Run(void)
{
    Board_Init();
    motor_test_request=0;
    MotorTest test={0};
    Encoder encoder;
    CarGeometry geometry=CarGeometry_Default();
    /* Local metrology only: this image has no course controller. Normal
     * firmware stays locked. Use measured geometry to calculate wheel speed. */
    geometry.confirmed=true;
    if (!CarGeometry_Valid(&geometry)) Error_Handler();
    Encoder_Init(&encoder,HAL_GetTick());
#if CAR_GROUND_TEST_BUILD
    GroundTest ground={0};
#else
    MotionControl motion;
    Motion_Reset(&motion);
    uint16_t closed_left=0,closed_right=0;
#endif
    uint32_t reset_flags=RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    WatchdogStart();
    uint32_t fed_at=HAL_GetTick();
    uint32_t trace_at=0;
    motor_test_diagnostics.magic=CAR_GROUND_TEST_BUILD ? 0x47545354U : 0x4D545354U;
    while (1) {
        /* OpenOCD's default examine event sets this bit on attachment. Clear
         * it before every motor command, not only at firmware startup. */
        DBGMCU->CR &= ~DBGMCU_CR_DBG_IWDG_STOP;
        uint32_t now=HAL_GetTick();
        bool enabled=Board_MotionEnabled(), pressed=Board_ButtonPressed();
        uint32_t request=motor_test_request;
        if (request) motor_test_request=0;
#if !CAR_GROUND_TEST_BUILD
        uint32_t previous_state=test.state;
#endif
        MotorTest_Update(&test,now,request,enabled,pressed);
        uint32_t encoder_at=encoder.last_at;
        Encoder_Update(&encoder,&geometry,now);
#if CAR_GROUND_TEST_BUILD
        GroundTest_Feedback(&ground,&test,now,encoder.valid,
            encoder.left_mm,encoder.right_mm,encoder.left_speed,encoder.right_speed,
            encoder.last_at!=encoder_at ? (float)(now-encoder_at)*0.001f : 0);
        ground_test_fault=ground.fault;
#else
        if (test.state==MOTOR_TEST_RUNNING && test.target_mm_s) {
            if (previous_state!=MOTOR_TEST_RUNNING) {
                Motion_Reset(&motion); closed_left=closed_right=0;
            }
            if (!encoder.valid) {
                test.state=MOTOR_TEST_ABORTED;
                closed_left=closed_right=0;
            } else if (encoder.last_at!=encoder_at) {
                Motion_Update(&motion,test.wheel==1 ? test.target_mm_s : 0,
                    test.wheel==2 ? test.target_mm_s : 0,
                    encoder.left_speed,encoder.right_speed,
                    (float)(now-encoder_at)*0.001f,&closed_left,&closed_right);
            }
            test.left_pwm=closed_left; test.right_pwm=closed_right;
        }
#endif
        Board_MotorApply(test.left_pwm,test.right_pwm);
        motor_test_diagnostics.now_ms=now;
        motor_test_diagnostics.state=test.state;
        motor_test_diagnostics.wheel=test.wheel;
        motor_test_diagnostics.started_at=test.started_at;
        motor_test_diagnostics.reset_flags=reset_flags;
        motor_test_diagnostics.enabled=enabled;
        motor_test_diagnostics.button_pressed=pressed;
        motor_test_diagnostics.left_pwm=test.left_pwm;
        motor_test_diagnostics.right_pwm=test.right_pwm;
        motor_test_diagnostics.encoder_a_counts=encoder.counts_a;
        motor_test_diagnostics.encoder_b_counts=encoder.counts_b;
        if (test.wheel && now-test.started_at<=test.duration_ms+300U &&
            motor_test_trace_count<256U &&
            (!motor_test_trace_count || now-trace_at>=MotorTest_TracePeriodMs(&test))) {
            trace_at=now;
            motor_test_trace[motor_test_trace_count]=(MotorTestTrace){
                now,(int32_t)encoder.counts_a,(int32_t)encoder.counts_b,
                test.left_pwm,test.right_pwm};
            ++motor_test_trace_count;
        }
        Board_SetLeds(test.state==MOTOR_TEST_RUNNING,true);
        /* A frozen SysTick cannot keep refreshing the watchdog. */
        if (now!=fed_at) { IWDG->KR=0xAAAAU; fed_at=now; }
        HAL_Delay(1);
    }
}
#endif
