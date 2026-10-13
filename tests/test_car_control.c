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
        CarControl_Update(&sim->car, sim->now, mask, pressed, true, true);
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
    Press(sim, 0xFFU);
    Release(sim, 0xFFU);
    Advance(sim, CAR_MARKER_CLEAR_MS + 10U, mask, false);
    assert(sim->car.state == CAR_RUNNING && sim->car.completed_laps == 0U);
}

static void TestBootAndReset(void)
{
    Simulation sim;
    Init(&sim, 0U, false);
    Advance(&sim, 100U, 0xFFU, false);
    assert(sim.car.state == CAR_STOPPED);
    Init(&sim, sim.now, true);
    Advance(&sim, 100U, 0xFFU, true);
    assert(sim.car.state == CAR_STOPPED); /* Reset 时按住按钮不会自动启动。 */
    Release(&sim, 0xFFU);
    Press(&sim, 0xFFU);
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
        Advance(&sim, 3U, 0xFFU, true);
        Advance(&sim, 3U, 0xFFU, false);
    }
    assert(sim.car.state == CAR_STOPPED);
    Press(&sim, 0xFFU);
    Advance(&sim, 1000U, 0xFFU, true);
    assert(sim.car.state == CAR_RUNNING); /* 长按只能启动一次。 */
    Advance(&sim, 3U, 0xFFU, false);
    Advance(&sim, 30U, 0xFFU, true);
    assert(sim.car.state == CAR_RUNNING); /* 松手抖动不能重新触发。 */
    Release(&sim, 0xFFU);
    Press(&sim, 0xFFU);
    assert(sim.car.state == CAR_STOPPED);
    Advance(&sim, 1000U, 0xFFU, true);
    assert(sim.car.state == CAR_STOPPED);
}

static void TestSteering(void)
{
    Simulation sim;
    Start(&sim, 0x18U);
    assert(sim.car.left_pwm == sim.car.right_pwm);
    Advance(&sim, 50U, 0x04U, false);
    assert(sim.car.left_pwm < sim.car.right_pwm);
    Advance(&sim, 50U, 0x20U, false);
    assert(sim.car.left_pwm > sim.car.right_pwm);
    Advance(&sim, 50U, 0x06U, false);
    assert(sim.car.left_pwm < sim.car.right_pwm);
    Advance(&sim, 50U, 0x60U, false);
    assert(sim.car.left_pwm > sim.car.right_pwm);
    Advance(&sim, 50U, 0x18U, false);
    assert(sim.car.left_pwm == sim.car.right_pwm);

}

static void TestRecoveryAndFault(void)
{
    Simulation sim;
    Start(&sim, 0x04U);
    Advance(&sim, 100U, 0U, false);
    assert(sim.car.state == CAR_RUNNING && sim.car.line_missing);
    assert(sim.car.left_pwm < sim.car.right_pwm);
    Advance(&sim, 100U, 0x18U, false);
    assert(!sim.car.line_missing && sim.car.state == CAR_RUNNING);
    Advance(&sim, CAR_LINE_LOST_TIMEOUT_MS + 20U, 0U, false);
    assert(sim.car.state == CAR_FAULT_LINE_LOST);
    Advance(&sim, 100U, 0x18U, false);
    assert(sim.car.state == CAR_FAULT_LINE_LOST); /* 重新看到线不会擅自启动。 */
    Press(&sim, 0xFFU);
    assert(sim.car.state == CAR_RUNNING);
    assert(sim.car.left_pwm == sim.car.right_pwm);
    Release(&sim, 0x20U);
    Advance(&sim, 100U, 0U, false);
    assert(sim.car.left_pwm > sim.car.right_pwm);
    Press(&sim, 0U);
    assert(sim.car.state == CAR_STOPPED); /* 找线过程中仍可按键停车。 */
}

static void TestAllSensorPatterns(void)
{
    Simulation sim;
    for (unsigned mask = 1U; mask <= 255U; ++mask)
    {
        Start(&sim, (uint8_t)mask);
        assert(sim.car.state == CAR_RUNNING);
        assert(sim.car.left_pwm > 0U || sim.car.right_pwm > 0U);
    }
}

#if CAR_TARGET_LAPS > 0U
static void CompleteLap(Simulation *sim)
{
    Advance(sim, CAR_MIN_LAP_TIME_MS + CAR_MARKER_CLEAR_MS, 0x18U, false);
    assert(sim->car.lap_armed);
    Advance(sim, CAR_MARKER_DEBOUNCE_MS + 2U, 0xFFU, false);
}

static void TestFinishLine(void)
{
    Simulation sim;
    Start(&sim, 0xFFU);
    Advance(&sim, 2000U, 0xFFU, false);
    assert(sim.car.completed_laps == 0U); /* 起点停留不计圈。 */
    for (unsigned lap = 1U; lap <= CAR_TARGET_LAPS; ++lap)
    {
        Advance(&sim, CAR_MIN_LAP_TIME_MS + CAR_MARKER_CLEAR_MS, 0x18U, false);
        assert(sim.car.lap_armed);
        for (unsigned bounce = 0U; bounce < 8U; ++bounce)
        {
            Advance(&sim, 3U, 0xFFU, false);
            Advance(&sim, 3U, 0x18U, false);
        }
        assert(sim.car.completed_laps == lap - 1U); /* 短促全黑不算圈。 */
        Advance(&sim, CAR_MARKER_DEBOUNCE_MS + 2U, 0xFFU, false);
        assert(sim.car.completed_laps == lap);
        assert(sim.car.state == (lap == CAR_TARGET_LAPS ? CAR_DONE : CAR_RUNNING));
        Advance(&sim, 1500U, 0xFFU, false);
        assert(sim.car.completed_laps == lap); /* 同次横线不能反复计数。 */
    }
    Advance(&sim, 100U, 0x18U, false);
    assert(sim.car.state == CAR_DONE); /* 到终点后保持停车。 */
    Press(&sim, 0xFFU);
    assert(sim.car.state == CAR_RUNNING && sim.car.completed_laps == 0U);
    Release(&sim, 0xFFU);
}

static void TestStartPositionAndDeparture(void)
{
    Simulation sim;
    Init(&sim, 0U, false);
    Press(&sim, 0x18U);
    assert(sim.car.state == CAR_START_REQUIRED);
    Release(&sim, 0xFFU);
    assert(sim.car.state == CAR_START_REQUIRED); /* 回起点不会自动启动。 */
    Press(&sim, 0xFFU);
    assert(sim.car.state == CAR_RUNNING);
    Release(&sim, 0xFFU);
    Advance(&sim, CAR_MARKER_CLEAR_MS - 1U, 0x18U, false);
    Advance(&sim, CAR_MARKER_DEBOUNCE_MS + 2U, 0xFFU, false);
    assert(!sim.car.lap_armed && sim.car.completed_laps == 0U);
    Advance(&sim, CAR_LINE_LOST_TIMEOUT_MS / 2U, 0U, false);
    assert(!sim.car.lap_armed); /* 全白丢线不能作为离开起点的证据。 */
    Advance(&sim, CAR_MARKER_DEBOUNCE_MS + 2U, 0xFFU, false);
    assert(sim.car.completed_laps == 0U);
    Advance(&sim, CAR_MARKER_CLEAR_MS + 10U, 0x18U, false);
    assert(sim.car.lap_armed);
    Press(&sim, 0x18U);
    assert(sim.car.state == CAR_STOPPED); /* 运行中可手动停车。 */
}

static void TestEarlyCrossing(void)
{
    Simulation sim;
    Start(&sim, 0x18U);
    Advance(&sim, CAR_MARKER_DEBOUNCE_MS + 2U, 0xFFU, false);
    assert(sim.car.completed_laps == 0U && !sim.car.lap_armed);
    Advance(&sim, CAR_MIN_LAP_TIME_MS + 100U, 0xFFU, false);
    assert(sim.car.completed_laps == 0U); /* 太早返回后停留不能补算一圈。 */
    CompleteLap(&sim);
    assert(sim.car.completed_laps == 1U);
}

static void TestClockWrap(void)
{
    Simulation sim;
    Init(&sim, UINT32_MAX - 10U, false);
    Press(&sim, 0xFFU);
    assert(sim.now < 100U && sim.car.state == CAR_RUNNING);
    Release(&sim, 0x18U);
    for (unsigned lap = 0U; lap < CAR_TARGET_LAPS; ++lap) { CompleteLap(&sim); }
    assert(sim.car.state == CAR_DONE && sim.car.completed_laps == CAR_TARGET_LAPS);

    Init(&sim, UINT32_MAX - 350U, false);
    Press(&sim, 0xFFU);
    Release(&sim, 0x20U);
    Advance(&sim, CAR_LINE_LOST_TIMEOUT_MS + 20U, 0U, false);
    assert(sim.now < 1000U && sim.car.state == CAR_FAULT_LINE_LOST);
}

#endif

static void TestCommunicationAndEnable(void)
{
    Simulation sim;
    Start(&sim, 0x18U);
    CarControl_Update(&sim.car, ++sim.now, 0xFFU, false, false, true);
    assert(sim.car.state == CAR_FAULT_SENSOR);
    assert(sim.car.left_pwm == 0U && sim.car.right_pwm == 0U);
    Advance(&sim, 100U, 0xFFU, false);
    assert(sim.car.state == CAR_FAULT_SENSOR); /* link recovery never restarts motion */
    Press(&sim, 0xFFU);
    assert(sim.car.state == CAR_RUNNING);
    Release(&sim, 0x18U);
    CarControl_Update(&sim.car, ++sim.now, 0x18U, false, true, false);
    assert(sim.car.state == CAR_STOPPED);
    assert(sim.car.left_pwm == 0U && sim.car.right_pwm == 0U);
    Advance(&sim, 100U, 0xFFU, false);
    assert(sim.car.state == CAR_STOPPED); /* switch-on never restarts motion */
    for (unsigned i = 0U; i < CAR_BUTTON_DEBOUNCE_MS + 2U; ++i)
        CarControl_Update(&sim.car, ++sim.now, 0xFFU, true, false, true);
    Advance(&sim, 100U, 0xFFU, true);
    assert(sim.car.state != CAR_RUNNING); /* holding a press during outage is consumed */
    Release(&sim, 0xFFU);
    Press(&sim, 0xFFU);
    assert(sim.car.state == CAR_RUNNING);
}

int main(void)
{
    TestBootAndReset();
    TestButton();
    TestSteering();
    TestRecoveryAndFault();
    TestAllSensorPatterns();
    TestCommunicationAndEnable();
#if CAR_TARGET_LAPS > 0U
    TestFinishLine();
    TestStartPositionAndDeparture();
    TestEarlyCrossing();
    TestClockWrap();
#endif
    printf("PASS: 8-channel control, buttons/reset, steering, all masks, communication/enable safety, %u-lap mode\n", (unsigned)CAR_TARGET_LAPS);
    return 0;
}
