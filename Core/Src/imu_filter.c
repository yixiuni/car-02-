#include "imu_filter.h"
#include "motion_control.h"
#include "car_config.h"
#include <math.h>
void ImuFilter_Init(ImuFilter *f) { *f=(ImuFilter){0}; }
void ImuFilter_Update(ImuFilter *f,uint32_t now,float z,bool ok,bool still)
{
    uint32_t dt=now-f->last_at;
    if (!ok || !isfinite(z) || fabsf(z)>490 || (f->seen && dt>CAR_CONTROL_MAX_GAP_MS)) {
        /* A gap invalidates integrated heading. Recalibrate stationary and
         * require a fresh manual task start; never resume an old path. */
        ImuFilter_Init(f); f->last_at=now; return;
    }
    f->online=true; f->rate_dps=z; f->last_at=now;
    if (!f->seen) { f->seen=true; return; }
    if (dt==0) return;
    if (!f->ready) {
        if (!still || fabsf(z)>10) { f->count=0; f->mean=f->m2=0; return; }
        ++f->count;
        float delta=z-f->mean;
        f->mean+=delta/(float)f->count; f->m2+=delta*(z-f->mean);
        if (f->count>=CAR_GYRO_CALIBRATION_SAMPLES) {
            if (f->m2/(float)(f->count-1)<=CAR_GYRO_CALIBRATION_STD_DPS*CAR_GYRO_CALIBRATION_STD_DPS) {
                f->bias_dps=f->mean; f->yaw_rad=0; f->ready=true;
            } else { f->count=0; f->mean=f->m2=0; }
        }
        return;
    }
    f->yaw_rad+=(z-f->bias_dps)*(CAR_PI/180.0f)*(float)dt*0.001f;
}
