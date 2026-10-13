#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    CAR_STOPPED = 0,
    CAR_RUNNING,
    CAR_FAULT_LINE_LOST,
    CAR_DONE,
    CAR_START_REQUIRED
} CarState;

typedef struct
{
    CarState state;
    uint16_t left_pwm;
    uint16_t right_pwm;
    int16_t previous_error;
    int8_t last_direction;
    bool previous_error_valid;
    bool line_missing;
    bool button_raw;
    bool button_stable;
    bool button_armed;
    uint8_t completed_laps;
    bool marker_raw;
    bool marker_stable;
    bool marker_clear_pending;
    bool lap_armed;
    uint32_t button_changed_at;
    uint32_t last_control_at;
    uint32_t line_missing_at;
    uint32_t marker_changed_at;
    uint32_t marker_clear_since;
    uint32_t lap_started_at;
} CarControl;

/* 纯控制逻辑，不依赖 HAL；pressed=true 表示按键按下。 */
void CarControl_Init(CarControl *car, uint32_t now_ms, bool pressed);
/* black_mask 的 bit0..bit4 按最左到最右排列，1 表示检测到黑线。 */
void CarControl_Update(CarControl *car, uint32_t now_ms,
                       uint8_t black_mask, bool pressed);

#ifdef __cplusplus
}
#endif

#endif /* CAR_CONTROL_H */
