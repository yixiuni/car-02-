#include "car_config.h"
#if CAR_SPEED_TRACE_BUILD
#include "speed_trace.h"
#include <math.h>
_Static_assert(sizeof(SpeedTraceSample)==28,"Speed trace decoder ABI");
_Static_assert(sizeof(SpeedTraceMetadata)==36,"Speed metadata decoder ABI");
volatile SpeedTraceMetadata speed_trace_meta;
volatile SpeedTraceSample speed_trace_samples[2][SPEED_TRACE_CAPACITY];
static uint32_t sampled_at[2];
static bool started[2];

void SpeedTrace_Reset(void)
{
    /* Counts delimit valid data. Do not clear 6 KB during button handling. */
    speed_trace_meta=(SpeedTraceMetadata){.magic=SPEED_TRACE_MAGIC,
        .record_size=sizeof(SpeedTraceSample),.capacity=SPEED_TRACE_CAPACITY};
    for(unsigned i=0;i<2;++i) {sampled_at[i]=0;started[i]=false;}
}
static int16_t Quantize(float x,uint8_t *flags)
{
    if(!isfinite(x)) {*flags|=SPEED_TRACE_CLIPPED;return 0;}
    if(x>32767) {*flags|=SPEED_TRACE_CLIPPED;return 32767;}
    if(x<-32768) {*flags|=SPEED_TRACE_CLIPPED;return -32768;}
    return (int16_t)(x+(x>=0 ? .5f : -.5f));
}
void SpeedTrace_Observe(const CarControl *c,const CarInputs *in)
{
    for(unsigned i=0;i<2;++i) {
        if(speed_trace_meta.done_mask&(1U<<i)) continue;
        bool selected=c->state==CAR_STRAIGHT && c->completed_laps==0 && c->segment==2*i;
        if(!started[i] && !selected) continue;
        bool first=!started[i],last=!selected;
        started[i]=true;
        if(!first && !last && in->now_ms-sampled_at[i]<SPEED_TRACE_PERIOD_MS) continue;
        sampled_at[i]=in->now_ms;
        uint32_t count=speed_trace_meta.count[i],index=count;
        if(count>=SPEED_TRACE_CAPACITY) {
            ++speed_trace_meta.dropped[i];
            if(!last) continue;
            index=SPEED_TRACE_CAPACITY-1; /* always retain terminal state */
        }
        SpeedTraceSample s={.tick=in->now_ms,.pwm={c->left_pwm,c->right_pwm},
            .state=(uint8_t)c->state,.segment=c->segment,.events=(uint8_t)c->point_events,
            .flags=(first?SPEED_TRACE_FIRST:0)|(last?SPEED_TRACE_LAST:0)};
        s.requested[0]=Quantize(c->left_target,&s.flags);s.requested[1]=Quantize(c->right_target,&s.flags);
        s.setpoint[0]=Quantize(c->motion.left.setpoint,&s.flags);s.setpoint[1]=Quantize(c->motion.right.setpoint,&s.flags);
        s.measured[0]=Quantize(in->left_speed,&s.flags);s.measured[1]=Quantize(in->right_speed,&s.flags);
        s.integral[0]=Quantize(c->motion.left.integral,&s.flags);s.integral[1]=Quantize(c->motion.right.integral,&s.flags);
        speed_trace_samples[i][index]=s;
        if(count<SPEED_TRACE_CAPACITY) speed_trace_meta.count[i]=count+1;
        if(last) speed_trace_meta.done_mask|=1U<<i;
    }
}
#endif
