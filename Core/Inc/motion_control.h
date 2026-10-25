#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H
#include <stdbool.h>
#include <stdint.h>
#define CAR_PI 3.14159265358979323846f

typedef struct {
    float diameter_mm, track_mm, counts_a_rev, counts_b_rev;
    float front_mm, rear_mm, half_width_mm, sensor_forward_mm;
    bool confirmed;
} CarGeometry;
typedef struct { float setpoint, integral; } WheelController;
typedef struct { WheelController left, right; } MotionControl;
CarGeometry CarGeometry_Default(void);
bool CarGeometry_Valid(const CarGeometry *g);
float Motion_Wrap(float angle);
void Motion_Reset(MotionControl *m);
void Motion_UpdateWithAccel(MotionControl *m, float left_target, float right_target,
                   float left_measured, float right_measured, float dt, float accel,
                   uint16_t *left_pwm, uint16_t *right_pwm);
void Motion_Update(MotionControl *m, float left_target, float right_target,
                   float left_measured, float right_measured, float dt,
                   uint16_t *left_pwm, uint16_t *right_pwm);
int32_t Encoder_CounterDelta(uint16_t current, uint16_t previous);
#endif
