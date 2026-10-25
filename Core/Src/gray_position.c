#include "gray_position.h"
#include <math.h>
#include <stddef.h>

bool GrayPosition_Estimate(const uint16_t raw[8],
    const GrayPositionCalibration *cal,GrayPosition *out)
{
    if(!out) return false;
    *out=(GrayPosition){.status=GRAY_POSITION_BAD_CALIBRATION};
    if(!raw || !cal || !isfinite(cal->pitch_mm) || cal->pitch_mm<=0 || cal->pitch_mm>40) return false;
    for(unsigned i=0;i<8;++i) {
        if(!isfinite(cal->white[i]) || !isfinite(cal->black[i]) ||
            fabsf(cal->black[i]-cal->white[i])<128.0f) return false;
    }
    unsigned islands=0,count=0,first=8,last=0;bool previous=false;
    for(unsigned i=0;i<8;++i) {
        float normalized=((float)raw[i]-cal->white[i])/(cal->black[i]-cal->white[i]);
        if(normalized>1.0f) out->clipped_mask|=(uint8_t)(1U<<i);
        normalized=fminf(1.0f,fmaxf(0.0f,normalized));
        /* Remove a small normalized white-floor band. These initial quality
         * limits are hypotheses to verify against a real lateral scan. */
        float weight=fmaxf(0.0f,(normalized-0.08f)/0.92f);
        out->weights[i]=weight;out->strength+=weight;
        out->peak=fmaxf(out->peak,weight);
        bool active=weight>=0.20f;
        if(active) {
            out->active_mask|=(uint8_t)(1U<<i);++count;
            if(first==8) first=i;
            last=i;
            if(!previous)++islands;
        }
        previous=active;
    }
    if(out->peak<0.25f || out->strength<0.25f) {out->status=GRAY_POSITION_WHITE;return false;}
    if(count>3 || out->strength>3.0f) {out->status=GRAY_POSITION_WIDE;return false;}
    if(islands!=1) {out->status=GRAY_POSITION_SPLIT;return false;}
    /* Distant sub-threshold background must not pull a narrow line toward
     * the opposite edge. Keep the single active island and its immediate
     * neighbors so a weak approaching channel still provides interpolation.
     * Strong separated islands remain ambiguous and were rejected above. */
    unsigned begin=first?first-1:0,end=last<7?last+1:7;
    float moment=0,second=0,mass=0;
    for(unsigned i=begin;i<=end;++i) {
        float x=((float)i-3.5f)*cal->pitch_mm;
        if(cal->reversed) x=-x;
        float weight=out->weights[i];
        mass+=weight;moment+=weight*x;second+=weight*x*x;
    }
    out->position_mm=moment/mass;
    out->spread_mm2=fmaxf(0.0f,second/mass-out->position_mm*out->position_mm);
    out->status=GRAY_POSITION_OK;
    return true;
}
