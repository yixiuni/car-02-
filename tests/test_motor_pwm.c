#include "motor_pwm.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    const unsigned duty[]={0,1,1260,3599,3600,4000};
    for (unsigned slow=0;slow<2;++slow) for (unsigned forward=0;forward<2;++forward)
    for (unsigned i=0;i<sizeof(duty)/sizeof(duty[0]);++i) {
        MotorPwm p=MotorPwm_Encode(duty[i],3600,forward,slow);
        unsigned drive=0,brake=0,coast=0;
        for (unsigned phase=0;phase<3600;++phase) {
            bool in1=phase<p.in1,in2=phase<p.in2;
            if (in1!=in2) { assert(in1==(bool)forward); ++drive; }
            else if (in1) ++brake;
            else ++coast;
        }
        unsigned expected=duty[i]>3600 ? 3600 : duty[i];
        assert(drive==expected);
        if (!expected) { assert(coast==3600); assert(!p.in1 && !p.in2); }
        else if (slow) assert(brake==3600-drive && coast==0);
        else assert(coast==3600-drive && brake==0);
    }
    MotorPwm p=MotorPwm_Encode(350,0,true,true);
    assert(p.in1==0 && p.in2==0);
    p=MotorPwm_Brake(3600,true);
    for(unsigned phase=0;phase<3600;phase++) {
        assert(phase<p.in1 && phase<p.in2); /* 1/1: brake, neither direction */
    }
    p=MotorPwm_Brake(3600,false);assert(!p.in1 && !p.in2);
    p=MotorPwm_Brake(0,true);assert(!p.in1 && !p.in2);
    puts("PASS: AT8236 truth table, effective duty, both directions, decay modes, zero and clamp");
}
