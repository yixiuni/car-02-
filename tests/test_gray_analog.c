#include "gray_analog.h"
#include "gray_calibration.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(void)
{
    /* Fixed independent protocol example: values 0,1,255,256,4095,32768,65535,0xAA55. */
    uint8_t b[21]={0x55,0xAA,2,16,0,0,1,0,255,0,0,1,255,15,0,128,255,255,85,170,0};
    unsigned sum=0;for(unsigned i=2;i<20;++i)sum+=b[i];b[20]=(uint8_t)~sum;
    uint16_t out[8]={0};const uint16_t expected[]={0,1,255,256,4095,32768,65535,0xAA55};
    assert(GrayAnalog_Decode(b,21,2,out));assert(!memcmp(out,expected,sizeof out));
    for(unsigned n=0;n<21;++n)assert(!GrayAnalog_Decode(b,n,2,out));
    assert(!GrayAnalog_Decode(b,22,2,out));
    assert(!GrayAnalog_Decode(b,21,3,out));
    for(unsigned i=0;i<21;++i){b[i]^=1;assert(!GrayAnalog_Decode(b,21,2,out));b[i]^=1;}
    b[2]=3;--b[20];assert(GrayAnalog_Decode(b,21,3,out));
    assert(!GrayAnalog_Decode(NULL,21,3,out));assert(!GrayAnalog_Decode(b,21,3,NULL));
    b[2]=2;++b[20];
    for(unsigned wrap=0;wrap<2;++wrap){
        GrayAnalogReceiver r;uint32_t start=wrap?UINT32_MAX-10U:0;
        GrayAnalog_Init(&r,start);
        assert(!GrayAnalog_CanRequest(&r,start+9));
        assert(GrayAnalog_CanRequest(&r,start+10));
        GrayAnalog_BeginRequest(&r,start+10);
        for(unsigned i=0;i<20;++i)GrayAnalog_OnByte(&r,start+11+i/11,b[i],false);
        assert(!r.valid && r.samples==0);
        GrayAnalog_OnByte(&r,start+12,b[20],false);
        assert(r.valid && r.samples==1 && !r.waiting);
        assert(!memcmp(r.values,expected,sizeof expected));
        GrayAnalog_OnByte(&r,start+15,0,false);assert(!r.valid && r.unexpected_bytes==1);
        GrayAnalog_BeginRequest(&r,start+30);
        for(unsigned i=0;i<20;++i)GrayAnalog_OnByte(&r,start+31,b[i],false);
        GrayAnalog_Poll(&r,start+50);assert(!r.valid && r.timeouts==1 && r.count==0);
        GrayAnalog_BeginRequest(&r,start+70);
        for(unsigned i=0;i<21;++i)GrayAnalog_OnByte(&r,start+71,b[i],false);
        assert(r.valid && r.samples==2);
        GrayAnalog_OnByte(&r,start+72,0,false);assert(!r.valid);
        GrayAnalog_BeginRequest(&r,start+100);
        for(unsigned i=0;i<21;++i)GrayAnalog_OnByte(&r,start+101,b[i]^(i==5),false);
        assert(!r.valid && r.errors==1);
        GrayAnalog_BeginRequest(&r,start+130);
        GrayAnalog_OnByte(&r,start+131,0x55,true);assert(!r.valid && r.errors==2);
        /* No query, no quiet interval: continuous packets must recover and
         * publish coherent values at no more than the legacy observation rate. */
        GrayAnalog_Init(&r,start);
        for(unsigned f=0;f<100;++f){
            for(unsigned i=0;i<21;++i)GrayAnalog_OnByte(&r,start+f*2,b[i],false);
            GrayAnalog_Poll(&r,start+f*2);
            assert(r.valid && r.count==0 && r.errors==0 && r.timeouts==0);
            assert(!memcmp(r.values,expected,sizeof expected));
        }
        assert(r.samples==17 && r.last_valid_at==start+192);
        GrayAnalog_Poll(&r,start+291);assert(r.valid);
        GrayAnalog_Poll(&r,start+292);assert(!r.valid);
        /* Attach at every possible packet boundary; never need a power cycle. */
        for(unsigned skip=0;skip<21;++skip){
            GrayAnalog_Init(&r,start);
            for(unsigned i=skip;i<21;++i)GrayAnalog_OnByte(&r,start,b[i],false);
            for(unsigned i=0;i<21;++i)GrayAnalog_OnByte(&r,start+2,b[i],false);
            assert(r.valid && !memcmp(r.values,expected,sizeof expected));
        }
        /* Every byte corruption rejects the damaged frame and re-synchronizes
         * to the following intact frame; threshold packets are never gray. */
        for(unsigned bad=0;bad<21;++bad){
            GrayAnalog_Init(&r,start);
            for(unsigned i=0;i<21;++i)GrayAnalog_OnByte(&r,start,b[i]^(i==bad),false);
            assert(!r.valid && r.samples==0);
            for(unsigned i=0;i<21;++i)GrayAnalog_OnByte(&r,start+2,b[i],false);
            assert(r.valid && r.samples==1);
        }
        GrayAnalog_Init(&r,start);
        b[2]=3;--b[20];
        for(unsigned i=0;i<21;++i)GrayAnalog_OnByte(&r,start,b[i],false);
        assert(!r.valid && r.samples==0);
        b[2]=2;++b[20];
        /* Partial automatic frames expire too, including across tick wrap. */
        GrayAnalog_Init(&r,start);
        for(unsigned i=0;i<10;++i)GrayAnalog_OnByte(&r,start,b[i],false);
        for(unsigned i=10;i<21;++i)GrayAnalog_OnByte(&r,start+20,b[i],false);
        assert(!r.valid && r.samples==0 && r.timeouts==1);
        for(unsigned i=0;i<21;++i)GrayAnalog_OnByte(&r,start+22,b[i],false);
        assert(r.valid && r.samples==1);
    }
    /* Bytes captured from this car while T321 rejected its continuous stream. */
    const uint8_t captured[]={0x55,0xaa,2,16,0xd0,2,0x1e,1,0x18,1,6,1,0x13,1,0x32,1,0x67,1,0x60,1,0xcc};
    GrayAnalogReceiver live;GrayAnalog_Init(&live,0);
    for(unsigned f=0;f<20;++f)
        for(unsigned i=0;i<sizeof captured;++i)GrayAnalog_OnByte(&live,f*2,captured[i],false);
    assert(live.valid && live.samples==4 && live.errors==0);
    assert(live.values[0]==720 && live.values[7]==352);
    /* Actual track captures: internal digital status was FF for both surfaces.
     * Calibrated analog readings must distinguish white, black and center line. */
    const uint16_t white[8]={1431,354,330,290,313,626,945,807};
    const uint16_t black[8]={2758,2154,2088,973,986,1610,2011,1549};
    const uint16_t center[8]={1334,351,343,1585,1693,679,859,710};
    assert(GrayAnalog_Classify(white,gray_calibration_threshold,gray_calibration_hysteresis,255,255)==0);
    assert(GrayAnalog_Classify(black,gray_calibration_threshold,gray_calibration_hysteresis,255,0)==255);
    assert(GrayAnalog_Classify(center,gray_calibration_threshold,gray_calibration_hysteresis,255,0)==0x18);
    uint16_t middle[8];for(unsigned i=0;i<8;++i)middle[i]=gray_calibration_threshold[i];
    assert(GrayAnalog_Classify(middle,gray_calibration_threshold,gray_calibration_hysteresis,255,0xA5)==0xA5);
    assert(GrayAnalog_Classify(white,gray_calibration_threshold,gray_calibration_hysteresis,0,0)==255);
    assert(GrayAnalog_Classify(black,gray_calibration_threshold,gray_calibration_hysteresis,0,255)==0);
    puts("PASS: queried/continuous analog frames, live UART replay, cadence, resynchronization, corruption/timeout/wrap, classification and hysteresis");
}
