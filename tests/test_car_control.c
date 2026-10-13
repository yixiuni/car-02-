#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "car_control.h"
#include "car_config.h"

typedef struct
{
    CarControl car;
    uint32_t now;
} Simulation;

static void Init(Simulation *sim, uint32_t now, bool held)
{
    sim->now = now;
    CarControl_Init(&sim->car, now, held);
}

static void Advance(Simulation *sim, uint32_t duration, uint8_t mask, bool pressed)
{
    for (uint32_t i = 0; i < duration; ++i)
    {
        ++sim->now;
        CarControl_Update(&sim->car, sim->now, mask, pressed);
        assert(sim->car.left_pwm <= CAR_MAX_SPEED);
        assert(sim->car.right_pwm <= CAR_MAX_SPEED);
        if (sim->car.state != CAR_RUNNING)
        {
            assert(sim->car.left_pwm == 0U && sim->car.right_pwm == 0U);
        }
    }
}

static void Press(Simulation *sim, uint8_t mask)
{
    Advance(sim, CAR_BUTTON_DEBOUNCE_MS + 2U, mask, true);
}

static void Release(Simulation *sim, uint8_t mask)
{
    Advance(sim, CAR_BUTTON_DEBOUNCE_MS + 2U, mask, false);
}

static void Start(Simulation *sim, uint8_t mask)
{
    Init(sim, 0U, false);
    Press(sim, mask);
    Release(sim, mask);
    assert(sim->car.state == CAR_RUNNING);
}

static void TestBootAndReset(void)
{
    Simulation sim;
    Init(&sim, 0U, false);
    Advance(&sim, 100U, 0x04U, false);
    assert(sim.car.state == CAR_STOPPED);
    Init(&sim, sim.now, true);
    Advance(&sim, 100U, 0x04U, true);
    assert(sim.car.state == CAR_STOPPED); /* Reset 时按住按钮不会自动启动。 */
    Release(&sim, 0x04U);
    Press(&sim, 0x04U);
    assert(sim.car.state == CAR_RUNNING);
    Init(&sim, sim.now, false);
    assert(sim.car.state == CAR_STOPPED);
    assert(sim.car.left_pwm == 0U && sim.car.right_pwm == 0U);
}

static void TestButton(void)
{
    Simulation sim;
    Init(&sim, 0U, false);
    for (unsigned i = 0U; i < 8U; ++i)
    {
        Advance(&sim, 3U, 0x04U, true);
        Advance(&sim, 3U, 0x04U, false);
    }
    assert(sim.car.state == CAR_STOPPED);
    Press(&sim, 0x04U);
    Advance(&sim, 1000U, 0x04U, true);
    assert(sim.car.state == CAR_RUNNING); /* 长按只能启动一次。 */
    Advance(&sim, 3U, 0x04U, false);
    Advance(&sim, 30U, 0x04U, true);
    assert(sim.car.state == CAR_RUNNING); /* 松手抖动不能重新触发。 */
    Release(&sim, 0x04U);
    Press(&sim, 0x04U);
    assert(sim.car.state == CAR_STOPPED);
    Advance(&sim, 1000U, 0x04U, true);
    assert(sim.car.state == CAR_STOPPED);
}

static void TestSteeringAndContinuousRun(void)
{
    Simulation sim;
    Start(&sim, 0x04U);
    assert(sim.car.left_pwm == sim.car.right_pwm);
    Advance(&sim, 50U, 0x02U, false);
    assert(sim.car.left_pwm < sim.car.right_pwm);
    Advance(&sim, 50U, 0x08U, false);
    assert(sim.car.left_pwm > sim.car.right_pwm);
    Advance(&sim, 50U, 0x03U, false);
    assert(sim.car.left_pwm < sim.car.right_pwm);
    Advance(&sim, 50U, 0x18U, false);
    assert(sim.car.left_pwm > sim.car.right_pwm);
    Advance(&sim, 50U, 0x04U, false);
    assert(sim.car.left_pwm == sim.car.right_pwm);
    for (unsigned lap = 0U; lap < 5U; ++lap)
    {
        Advance(&sim, 1000U, 0x04U, false);
        Advance(&sim, 100U, 0x1FU, false);
        assert(sim.car.state == CAR_RUNNING); /* 全黑不会触发一圈停车。 */
    }
}

static void TestRecoveryAndFault(void)
{
    Simulation sim;
    Start(&sim, 0x02U);
    Advance(&sim, 100U, 0U, false);
    assert(sim.car.state == CAR_RUNNING && sim.car.line_missing);
    assert(sim.car.left_pwm < sim.car.right_pwm);
    Advance(&sim, 100U, 0x04U, false);
    assert(!sim.car.line_missing && sim.car.state == CAR_RUNNING);
    Advance(&sim, CAR_LINE_LOST_TIMEOUT_MS + 20U, 0U, false);
    assert(sim.car.state == CAR_FAULT_LINE_LOST);
    Advance(&sim, 100U, 0x04U, false);
    assert(sim.car.state == CAR_FAULT_LINE_LOST); /* 重新看到线不会擅自启动。 */
    Press(&sim, 0x04U);
    assert(sim.car.state == CAR_RUNNING);
    assert(sim.car.left_pwm == sim.car.right_pwm);
    Release(&sim, 0x08U);
    Advance(&sim, 100U, 0U, false);
    assert(sim.car.left_pwm > sim.car.right_pwm);
    Press(&sim, 0U);
    assert(sim.car.state == CAR_STOPPED); /* 找线过程中仍可按键停车。 */
}

static void TestAllSensorPatterns(void)
{
    Simulation sim;
    for (uint8_t mask = 1U; mask < 32U; ++mask)
    {
        Start(&sim, mask);
        assert(sim.car.state == CAR_RUNNING);
        assert(sim.car.left_pwm > 0U || sim.car.right_pwm > 0U);
    }
}

static void TestClockWrap(void)
{
    Simulation sim;
    Init(&sim, UINT32_MAX - 10U, false);
    Press(&sim, 0x04U);
    assert(sim.now < 100U && sim.car.state == CAR_RUNNING);
    Release(&sim, 0x04U);
    Press(&sim, 0x04U);
    assert(sim.car.state == CAR_STOPPED);

    Init(&sim, UINT32_MAX - 350U, false);
    Press(&sim, 0x08U);
    Release(&sim, 0x08U);
    Advance(&sim, CAR_LINE_LOST_TIMEOUT_MS + 20U, 0U, false);
    assert(sim.now < 1000U && sim.car.state == CAR_FAULT_LINE_LOST);
}

int main(void)
{
    TestBootAndReset();
    TestButton();
    TestSteeringAndContinuousRun();
    TestRecoveryAndFault();
    TestAllSensorPatterns();
    TestClockWrap();
    puts("PASS: boot/reset, button debounce/hold, steering, continuous run, recovery/fault, all sensor patterns, tick wrap");
    return 0;
}
