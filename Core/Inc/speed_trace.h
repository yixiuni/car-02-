#ifndef SPEED_TRACE_H
#define SPEED_TRACE_H
#include "car_control.h"
#define SPEED_TRACE_CAPACITY 104U
#define SPEED_TRACE_PERIOD_MS 20U
#define SPEED_TRACE_MAGIC 0x53504431U
enum { SPEED_TRACE_CLIPPED=1U, SPEED_TRACE_FIRST=2U, SPEED_TRACE_LAST=4U };
typedef struct {
    uint32_t tick;
    int16_t requested[2],setpoint[2],measured[2],integral[2];
    uint16_t pwm[2];
    uint8_t state,segment,events,flags;
} SpeedTraceSample;
typedef struct {
    uint32_t magic,record_size,capacity,count[2],dropped[2],done_mask,max_cycles;
} SpeedTraceMetadata;
extern volatile SpeedTraceMetadata speed_trace_meta;
extern volatile SpeedTraceSample speed_trace_samples[2][SPEED_TRACE_CAPACITY];
/* Diagnostics only: never writes controller, inputs, motor or mode state. */
void SpeedTrace_Reset(void);
void SpeedTrace_Observe(const CarControl *car,const CarInputs *input);
#endif
