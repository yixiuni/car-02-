#include "gray_position.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    GrayPositionCalibration c={.pitch_mm=15};GrayPosition o;
    uint16_t v[8];
    for(unsigned i=0;i<8;++i){c.white[i]=200+50*i;c.black[i]=c.white[i]+1000;v[i]=(uint16_t)c.white[i];}
    assert(!GrayPosition_Estimate(v,&c,&o) && o.status==GRAY_POSITION_WHITE);
    v[3]+=1000;assert(GrayPosition_Estimate(v,&c,&o) && fabsf(o.position_mm+7.5f)<.001f);
    v[4]+=1000;assert(GrayPosition_Estimate(v,&c,&o) && fabsf(o.position_mm)<.001f);
    /* Sub-channel movement must retain information the binary mask discards. */
    v[3]-=250;assert(GrayPosition_Estimate(v,&c,&o));float right=o.position_mm;
    assert(right>0 && right<7.5f);
    c.reversed=true;assert(GrayPosition_Estimate(v,&c,&o) && fabsf(o.position_mm+right)<.001f);
    c.reversed=false;
    /* Captured edge readings had a distant white-floor residual. That must
     * not pull the estimated line tens of millimeters toward the center. */
    for(unsigned i=0;i<8;++i)v[i]=(uint16_t)c.white[i];
    v[0]+=240;v[7]+=700;
    assert(GrayPosition_Estimate(v,&c,&o) && fabsf(o.position_mm-52.5f)<.001f);
    v[6]+=240;
    assert(GrayPosition_Estimate(v,&c,&o) && o.position_mm<52.5f && o.position_mm>37.5f);
    v[0]+=200;
    assert(!GrayPosition_Estimate(v,&c,&o) && o.status==GRAY_POSITION_SPLIT);
    /* A continuous handoff between adjacent probes should be monotonic. */
    float before=-100;
    for(unsigned step=0;step<=100;++step) {
        for(unsigned i=0;i<8;++i)v[i]=(uint16_t)c.white[i];
        v[3]+=(uint16_t)(1000-10*step);v[4]+=(uint16_t)(10*step);
        assert(GrayPosition_Estimate(v,&c,&o));
        assert(o.position_mm>=before && o.position_mm>=-7.5f && o.position_mm<=7.5f);
        before=o.position_mm;
    }
    for(unsigned i=0;i<8;++i)v[i]=(uint16_t)c.black[i];
    assert(!GrayPosition_Estimate(v,&c,&o) && o.status==GRAY_POSITION_WIDE);
    for(unsigned i=0;i<8;++i)v[i]=(uint16_t)c.white[i];
    v[1]+=1000;v[6]+=1000;
    assert(!GrayPosition_Estimate(v,&c,&o) && o.status==GRAY_POSITION_SPLIT);
    /* Per-channel polarity inversion must preserve the spatial result. */
    for(unsigned i=0;i<8;++i){c.white[i]=3000;c.black[i]=1000;v[i]=3000;}
    v[4]=1000;assert(GrayPosition_Estimate(v,&c,&o) && fabsf(o.position_mm-7.5f)<.001f);
    c.black[2]=c.white[2];assert(!GrayPosition_Estimate(v,&c,&o) && o.status==GRAY_POSITION_BAD_CALIBRATION);
    c.black[2]=NAN;assert(!GrayPosition_Estimate(v,&c,&o));
    assert(!GrayPosition_Estimate(NULL,&c,&o));assert(!GrayPosition_Estimate(v,NULL,&o));
    assert(!GrayPosition_Estimate(v,&c,NULL));
    puts("PASS: continuous sub-channel position, per-channel normalization, polarity/order, white/wide/split rejection and invalid calibration");
}
