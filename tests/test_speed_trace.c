#include "speed_trace.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    CarControl c={0};CarInputs in={0};SpeedTrace_Reset();
    SpeedTrace_Observe(&c,&in);assert(speed_trace_meta.count[0]==0);
    c.state=CAR_STRAIGHT;c.point_events=1;c.left_target=800;c.right_target=801;
    c.motion.left.setpoint=600.4f;c.motion.right.setpoint=601.6f;
    c.motion.left.integral=-35.6f;c.motion.right.integral=40.4f;
    c.left_pwm=700;c.right_pwm=701;in.left_speed=555.2f;in.right_speed=556.8f;
    CarControl original=c;CarInputs original_in=in;
    SpeedTrace_Observe(&c,&in);
    assert(!memcmp(&c,&original,sizeof(c)) && !memcmp(&in,&original_in,sizeof(in)));
    assert(speed_trace_meta.count[0]==1 && speed_trace_meta.count[1]==0);
    SpeedTraceSample s=speed_trace_samples[0][0];
    assert(s.flags==SPEED_TRACE_FIRST && s.requested[0]==800 && s.setpoint[1]==602);
    assert(s.measured[1]==557 && s.integral[0]==-36 && s.pwm[1]==701);
    in.now_ms=19;SpeedTrace_Observe(&c,&in);assert(speed_trace_meta.count[0]==1);
    in.now_ms=20;SpeedTrace_Observe(&c,&in);assert(speed_trace_meta.count[0]==2);
    c.state=CAR_ARC;c.segment=1;in.now_ms=21;SpeedTrace_Observe(&c,&in);
    assert(speed_trace_meta.done_mask==1 && speed_trace_meta.count[0]==3);
    assert(speed_trace_samples[0][2].flags&SPEED_TRACE_LAST);
    for(unsigned i=0;i<200;++i){in.now_ms+=20;SpeedTrace_Observe(&c,&in);}
    assert(speed_trace_meta.count[0]==3 && speed_trace_meta.count[1]==0);
    c.state=CAR_STRAIGHT;c.segment=2;
    for(unsigned i=0;i<SPEED_TRACE_CAPACITY+10;++i){in.now_ms+=20;SpeedTrace_Observe(&c,&in);}
    assert(speed_trace_meta.count[1]==SPEED_TRACE_CAPACITY && speed_trace_meta.dropped[1]==10);
    c.state=CAR_FAULT;in.now_ms++;SpeedTrace_Observe(&c,&in);
    assert(speed_trace_meta.done_mask==3 && speed_trace_meta.dropped[1]==11);
    assert(speed_trace_samples[1][SPEED_TRACE_CAPACITY-1].state==CAR_FAULT);
    SpeedTraceMetadata frozen=speed_trace_meta;
    c.state=CAR_STRAIGHT;c.segment=0;c.completed_laps=1;in.now_ms+=500;SpeedTrace_Observe(&c,&in);
    assert(!memcmp(&frozen,(const void *)&speed_trace_meta,sizeof(frozen)));
    SpeedTrace_Reset();assert(speed_trace_meta.count[0]==0 && speed_trace_meta.done_mask==0);
    c.completed_laps=0;c.motion.left.integral=NAN;c.motion.right.integral=40000;
    in.now_ms=UINT32_MAX-9;SpeedTrace_Observe(&c,&in);
    assert(speed_trace_samples[0][0].flags&SPEED_TRACE_CLIPPED);
    assert(speed_trace_samples[0][0].integral[0]==0 && speed_trace_samples[0][0].integral[1]==32767);
    in.now_ms=10;SpeedTrace_Observe(&c,&in);assert(speed_trace_meta.count[0]==2);
    puts("PASS: speed trace preserves inputs, captures two windows, quantizes, freezes, bounds capacity, retains abort, resets and handles tick wrap");
}
