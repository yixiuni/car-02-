#include "motor_test.h"
#include <assert.h>
#include <stdio.h>

static void off(const MotorTest *t) { assert(t->left_pwm==0 && t->right_pwm==0); }
int main(void)
{
    for (unsigned request=17;request<=19;++request) {
        MotorTest t={0};
        MotorTest_Update(&t,1,request,true,false);
        assert(t.state==MOTOR_TEST_ABORTED); off(&t);
    }
    for (unsigned wheel=1;wheel<=2;++wheel) {
        MotorTest t={0};
        MotorTest_Update(&t,100,0,true,false); off(&t);
        MotorTest_Update(&t,100,wheel,true,false);
        assert(t.state==MOTOR_TEST_RUNNING);
        assert(t.left_pwm==(wheel==1 ? MOTOR_TEST_DUTY : 0));
        assert(t.right_pwm==(wheel==2 ? MOTOR_TEST_DUTY : 0));
        for (unsigned n=101;n<=400;++n) MotorTest_Update(&t,n,0,true,false);
        assert(t.state==MOTOR_TEST_DONE); off(&t);
        MotorTest_Update(&t,401,3-wheel,true,false); off(&t);
        assert(t.state==MOTOR_TEST_DONE);
    }
    for (unsigned reason=0;reason<3;++reason) {
        MotorTest t={0};
        MotorTest_Update(&t,10,1,true,false);
        MotorTest_Update(&t,reason==2 ? 61 : 11,0,reason!=0,reason==1);
        assert(t.state==MOTOR_TEST_ABORTED); off(&t);
        MotorTest_Update(&t,62,1,true,false); off(&t);
    }
    for (unsigned reason=0;reason<3;++reason) {
        MotorTest t={0};
        MotorTest_Update(&t,1,reason==0 ? 17 : 1,reason!=1,reason==2);
        assert(t.state==MOTOR_TEST_ABORTED); off(&t);
    }
    MotorTest wrap={0};
    MotorTest_Update(&wrap,UINT32_MAX-100,1,true,false);
    for (unsigned n=1;n<=300;++n) MotorTest_Update(&wrap,UINT32_MAX-100+n,0,true,false);
    assert(wrap.state==MOTOR_TEST_DONE); off(&wrap);
    for (unsigned request=3;request<=8;++request) {
        MotorTest t={0};
        unsigned duty=request<=4 ? 150 : request<=6 ? 250 : 350;
        for (unsigned n=0;n<1000;++n) {
            MotorTest_Update(&t,n,n==0 ? request : 0,true,false);
            assert(t.state==MOTOR_TEST_RUNNING);
            assert(t.left_pwm==((request&1U) ? duty : 0));
            assert(t.right_pwm==((request&1U) ? 0 : duty));
        }
        MotorTest_Update(&t,1000,0,true,false);
        assert(t.state==MOTOR_TEST_DONE); off(&t);
        MotorTest_Update(&t,1001,request,true,false); off(&t);
    }
    puts("PASS: one-shot motor timing, channels, inhibit, abort, gap and tick wrap");
    for (unsigned request=9;request<=16;++request) {
        MotorTest t={0};
        for (unsigned n=0;n<2000;++n) {
            MotorTest_Update(&t,n,n==0 ? request : 0,true,false);
            assert(t.state==MOTOR_TEST_RUNNING); off(&t);
            assert(t.target_mm_s==(request<=10 ? 150 : request<=12 ? 180 : request<=14 ? 85 : 250));
        }
        MotorTest_Update(&t,2000,0,true,false);
        assert(t.state==MOTOR_TEST_DONE); off(&t);
    }
}
