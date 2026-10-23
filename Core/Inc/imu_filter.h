#ifndef IMU_FILTER_H
#define IMU_FILTER_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint32_t count, last_at;
    float mean, m2, bias_dps, yaw_rad, rate_dps;
    bool ready, online, seen;
} ImuFilter;
void ImuFilter_Init(ImuFilter *filter);
void ImuFilter_Update(ImuFilter *filter, uint32_t now, float z_dps,
                      bool sample_ok, bool wheels_still);
#endif
