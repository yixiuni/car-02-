/* Host-only replay of the actual experimental estimator, not a second model. */
#include "gray_position.h"
#include <stdio.h>

int main(void)
{
    GrayPositionCalibration cal={0};
    unsigned reversed;
    for(unsigned i=0;i<8;++i) if(scanf("%f",&cal.white[i])!=1) return 2;
    for(unsigned i=0;i<8;++i) if(scanf("%f",&cal.black[i])!=1) return 2;
    if(scanf("%f %u",&cal.pitch_mm,&reversed)!=2 || reversed>1) return 2;
    cal.reversed=reversed!=0;
    for(;;) {
        uint16_t raw[8];unsigned value;
        int first=scanf("%u",&value);
        if(first==EOF) break;
        if(first!=1 || value>65535) return 2;
        raw[0]=(uint16_t)value;
        for(unsigned i=1;i<8;++i) {
            if(scanf("%u",&value)!=1 || value>65535) return 2;
            raw[i]=(uint16_t)value;
        }
        GrayPosition out;
        bool valid=GrayPosition_Estimate(raw,&cal,&out);
        printf("%u,%u,%.6f,%.6f,%.6f,%.6f,%u,%u\n",
            valid,(unsigned)out.status,(double)out.position_mm,
            (double)out.strength,(double)out.peak,(double)out.spread_mm2,
            out.active_mask,out.clipped_mask);
    }
    return ferror(stdin)?2:0;
}
