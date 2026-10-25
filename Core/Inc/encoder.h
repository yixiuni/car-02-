#ifndef ENCODER_H
#define ENCODER_H
#include "motion_control.h"
typedef struct {
    int64_t counts_a, counts_b;
    uint16_t previous_a, previous_b;
    uint32_t last_at;
    float left_mm, right_mm, left_speed, right_speed;
    bool valid;
} Encoder;
void Encoder_Init(Encoder *encoder, uint32_t now);
void Encoder_Update(Encoder *encoder, const CarGeometry *geometry, uint32_t now);
#endif
