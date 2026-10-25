#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H
#include "motion_control.h"
#include "car_config.h"

typedef enum {
    CAR_STOPPED, CAR_STRAIGHT, CAR_CAPTURE, CAR_ARC, CAR_BRAKING, CAR_DONE, CAR_FAULT
} CarState;
typedef enum {
    CAR_OK, CAR_FAULT_CONFIG, CAR_FAULT_GRAY, CAR_FAULT_IMU, CAR_FAULT_ENCODER,
    CAR_FAULT_TIMING, CAR_FAULT_STALL, CAR_FAULT_LINE, CAR_FAULT_ENDPOINT, CAR_FAULT_TIMEOUT
} CarFault;
typedef enum { CAR_POINT_A, CAR_POINT_B, CAR_POINT_C, CAR_POINT_D } CarPoint;
typedef struct {
    uint32_t now_ms, gray_samples;
    bool pressed, enabled, gray_online, imu_ready, encoder_valid;
    uint8_t black_mask;
    float yaw_rad, left_mm, right_mm, left_speed, right_speed;
} CarInputs;
typedef struct {
    CarGeometry geometry;
    MotionControl motion;
    CarState state;
    CarFault fault;
    uint8_t task, segment, completed_laps;
    uint8_t target_laps; /* 0 = legacy task default; selectable modes use 1..4 */
    CarPoint last_point;
    uint32_t point_events, mode_events;
    uint16_t left_pwm, right_pwm;
    float x_mm, y_mm, heading_rad, yaw_offset;
    float left_target, right_target, last_left_mm, last_right_mm, distance_mm;
    float segment_distance, arc_heading_start;
    float line_error, line_rate;
    uint32_t last_gray_samples, line_at, last_line_at, missing_at;
    uint8_t black_frames, white_frames;
    bool line_missing, have_line;
    uint32_t last_update, run_at, segment_at, brake_at, settled_at;
    uint32_t left_stall_at, right_stall_at;
    bool settled, left_stall, right_stall;
    bool button_raw, button_stable, button_armed, hold_consumed;
    uint32_t button_changed_at, pressed_at;
#if CAR_TIME_TRIAL_EDGE_POSE
    /* Latest centered black pose, then frozen black/white midpoint pose. */
    float edge_x, edge_y, edge_heading;
    bool edge_valid, edge_ready;
#endif
} CarControl;
void CarControl_Init(CarControl *car, const CarGeometry *geometry, uint32_t now, bool pressed);
void CarControl_Update(CarControl *car, const CarInputs *input);
bool CarControl_Active(const CarControl *car);
bool CarControl_BrakeActive(const CarControl *car, uint32_t now_ms);
#endif
