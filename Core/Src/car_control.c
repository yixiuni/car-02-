#include "car_control.h"
#include "car_config.h"

static uint16_t ClampSpeed(int32_t speed)
{
    if (speed < 0)
    {
        return 0U;
    }
    if (speed > CAR_MAX_SPEED)
    {
        return CAR_MAX_SPEED;
    }
    return (uint16_t)speed;
}

static void Stop(CarControl *car, CarState state)
{
    car->state = state;
    car->left_pwm = 0U;
    car->right_pwm = 0U;
    car->line_missing = false;
    car->previous_error_valid = false;
}

/* 非阻塞消抖；一次按下只触发一次，松手后才允许下一次触发。 */
static bool ButtonPressedEvent(CarControl *car, uint32_t now, bool pressed)
{
    if (pressed != car->button_raw)
    {
        car->button_raw = pressed;
        car->button_changed_at = now;
    }
    if ((uint32_t)(now - car->button_changed_at) < CAR_BUTTON_DEBOUNCE_MS ||
        car->button_stable == car->button_raw)
    {
        return false;
    }

    car->button_stable = car->button_raw;
    if (!car->button_stable)
    {
        car->button_armed = true;
        return false;
    }
    if (car->button_armed)
    {
        car->button_armed = false;
        return true;
    }
    return false;
}

void CarControl_Init(CarControl *car, uint32_t now_ms, bool pressed)
{
    *car = (CarControl){0};
    car->state = CAR_STOPPED;
    car->button_raw = pressed;
    car->button_stable = pressed;
    car->button_armed = !pressed;
    car->button_changed_at = now_ms;
    car->last_control_at = now_ms;
}

void CarControl_Update(CarControl *car, uint32_t now_ms,
                       uint8_t black_mask, bool pressed)
{
    static const int16_t weights[5] = {-200, -100, 0, 100, 200};
    int32_t sum = 0;
    int32_t count = 0;
    int32_t error;
    int32_t derivative;
    int32_t correction;
    int32_t magnitude;
    int32_t base;

    if (ButtonPressedEvent(car, now_ms, pressed))
    {
        if (car->state == CAR_RUNNING)
        {
            Stop(car, CAR_STOPPED);
        }
        else
        {
            /* 开始新一次运行时清除上次的偏差和丢线计时。 */
            car->state = CAR_RUNNING;
            car->previous_error = 0;
            car->previous_error_valid = false;
            car->last_direction = 0;
            car->line_missing = false;
            car->last_control_at = now_ms - CAR_CONTROL_PERIOD_MS;
        }
    }

    if (car->state != CAR_RUNNING ||
        (uint32_t)(now_ms - car->last_control_at) < CAR_CONTROL_PERIOD_MS)
    {
        return;
    }
    car->last_control_at = now_ms;
    black_mask &= 0x1FU;

    if (black_mask == 0U)
    {
        if (!car->line_missing)
        {
            car->line_missing = true;
            car->line_missing_at = now_ms;
        }
        if ((uint32_t)(now_ms - car->line_missing_at) >= CAR_LINE_LOST_TIMEOUT_MS)
        {
            /* 丢线过久锁定停车，重新放回黑线后按键才能启动。 */
            Stop(car, CAR_FAULT_LINE_LOST);
            return;
        }

        /* 短暂丢线：沿最后偏差方向低速找线，不倒车。 */
        car->left_pwm = (car->last_direction < 0) ? 0U : CAR_SEARCH_SPEED;
        car->right_pwm = (car->last_direction > 0) ? 0U : CAR_SEARCH_SPEED;
        car->previous_error_valid = false;
        return;
    }
    car->line_missing = false;

    /* 全黑区域只低速直行；本版本不把它当作终点或圈数标记。 */
    if (black_mask == 0x1FU)
    {
        car->left_pwm = CAR_WIDE_LINE_SPEED;
        car->right_pwm = CAR_WIDE_LINE_SPEED;
        car->previous_error_valid = false;
        return;
    }

    for (uint8_t i = 0U; i < 5U; ++i)
    {
        if ((black_mask & (1U << i)) != 0U)
        {
            sum += weights[i];
            ++count;
        }
    }
    error = sum / count;
    derivative = car->previous_error_valid ? error - car->previous_error : 0;
    correction = CAR_KP * error + CAR_KD * derivative;
    magnitude = (error < 0) ? -error : error;
    base = CAR_BASE_SPEED - magnitude / CAR_TURN_SLOWDOWN_DIVISOR;

    /* 黑线在右边：左轮加速、右轮减速；在左边则相反。 */
    car->left_pwm = ClampSpeed(base + correction);
    car->right_pwm = ClampSpeed(base - correction);
    car->previous_error = (int16_t)error;
    car->previous_error_valid = true;
    if (error != 0)
    {
        car->last_direction = (error < 0) ? -1 : 1;
    }
}
