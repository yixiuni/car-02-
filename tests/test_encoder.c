#include "encoder.h"
#include "car_config.h"
#include "stm32f1xx_hal.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

TIM_TypeDef test_tim4, test_tim8;

static void advance(Encoder *e, const CarGeometry *g, uint32_t dt,
                    int32_t forward_a, int32_t forward_b)
{
    TIM4->CNT=(uint16_t)(TIM4->CNT+forward_a*CAR_ENCODER_A_SIGN);
    TIM8->CNT=(uint16_t)(TIM8->CNT+forward_b*CAR_ENCODER_B_SIGN);
    Encoder_Update(e,g,e->last_at+dt);
}

int main(void)
{
    /* Exhaust all counter differences at several starting positions. */
    const uint16_t origins[]={0,1,32767,32768,65535};
    for (unsigned i=0;i<sizeof origins/sizeof origins[0];++i)
        for (int32_t delta=-32768;delta<=32767;++delta)
            assert(Encoder_CounterDelta((uint16_t)(origins[i]+delta),origins[i])==delta);

    CarGeometry g=CarGeometry_Default(); g.confirmed=false;
    Encoder e; Encoder_Init(&e,UINT32_MAX-20);
    int64_t expected_a=0,expected_b=0;
    /* Locked calibration must retain all counts, even with long loop gaps,
       reversals, repeated 16-bit timer wraps and the 32-bit tick wrap. */
    for (unsigned i=0;i<20000;++i) {
        int32_t da=(i%5==0) ? -11 : 23;
        int32_t db=(i%7==0) ? -9 : 19;
        uint32_t dt=(i%13==0) ? 71 : 7;
        advance(&e,&g,dt,da,db);
        expected_a+=da; expected_b+=db;
        assert(e.counts_a==expected_a && e.counts_b==expected_b);
        assert(!e.valid && e.left_speed==0 && e.right_speed==0);
    }

    /* A call before the sample deadline must defer, not discard, its pulses. */
    Encoder_Init(&e,0);
    advance(&e,&g,1,10,12);
    assert(e.counts_a==0 && e.counts_b==0 && e.last_at==0);
    advance(&e,&g,CAR_CONTROL_PERIOD_MS,20,21);
    assert(e.counts_a==30 && e.counts_b==33);

    /* Replay the two observed runs through the production accumulation code.
       These inputs are recorded counts, not an independent distance truth. */
    const int64_t starts[][2]={{-373,5581},{5849,11881}};
    const int32_t deltas[][2]={{6984,6926},{6917,6596}};
    const float mm[][2]={{971.69544f,959.96307f},{962.37362f,914.22415f}};
    g.diameter_mm=65.0f; /* Historical captures used the nominal diameter. */
    for (unsigned i=0;i<2;++i) {
        Encoder_Init(&e,0);
        e.counts_a=starts[i][0]; e.counts_b=starts[i][1];
        e.previous_a=(uint16_t)(starts[i][0]*CAR_ENCODER_A_SIGN);
        e.previous_b=(uint16_t)(starts[i][1]*CAR_ENCODER_B_SIGN);
        TIM4->CNT=e.previous_a; TIM8->CNT=e.previous_b;
        advance(&e,&g,60000,deltas[i][0],deltas[i][1]);
        assert(e.counts_a-starts[i][0]==deltas[i][0]);
        assert(e.counts_b-starts[i][1]==deltas[i][1]);
        assert(fabsf(deltas[i][0]*CAR_PI*g.diameter_mm/g.counts_a_rev-mm[i][0])<0.001f);
        assert(fabsf(deltas[i][1]*CAR_PI*g.diameter_mm/g.counts_b_rev-mm[i][1])<0.001f);
    }

    /* Speed validity gates must not erase raw position measurements. */
    Encoder_Init(&e,0); g=CarGeometry_Default(); g.confirmed=true;
    advance(&e,&g,10,10,10); assert(e.valid);
    advance(&e,&g,CAR_CONTROL_MAX_GAP_MS+1,17,21);
    assert(!e.valid && e.counts_a==27 && e.counts_b==31);
    advance(&e,&g,10,10,10); assert(e.valid);
    advance(&e,&g,10,1000,1000);
    assert(!e.valid && e.counts_a==1037 && e.counts_b==1041);
    advance(&e,&g,10,0,0); assert(e.valid);
    assert(fabsf(e.left_mm-1037*CAR_PI*g.diameter_mm/g.counts_a_rev)<0.001f);
    assert(fabsf(e.right_mm-1041*CAR_PI*g.diameter_mm/g.counts_b_rev)<0.001f);
    puts("PASS: production encoder accumulation, wrap, timing, locked calibration and recorded-run replay");
}
