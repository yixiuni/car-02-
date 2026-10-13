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
    car->lap_armed = false;
    car->marker_clear_pending = false;
}

/* 持续全黑才确认横线，只在确认进入横线时产生一次事件。 */
static bool MarkerEntered(CarControl *car, uint32_t now, uint8_t mask)
{
    bool present = mask == CAR_MARKER_MASK;
    if (present != car->marker_raw)
    {
        car->marker_raw = present;
        car->marker_changed_at = now;
    }
    if (car->marker_stable != car->marker_raw &&
        (uint32_t)(now - car->marker_changed_at) >= CAR_MARKER_DEBOUNCE_MS)
    {
        car->marker_stable = car->marker_raw;
        return car->marker_stable;
    }
    return false;
}

static void UpdateLaps(CarControl *car, uint32_t now, uint8_t mask, bool entered)
{
    /* 只有稳定回到普通黑线才确认离开横线，全白丢线不能用于重新许可。 */
    if (mask != 0U && mask != CAR_MARKER_MASK)
    {
        if (!car->marker_clear_pending)
        {
            car->marker_clear_pending = true;
            car->marker_clear_since = now;
        }
        if ((uint32_t)(now - car->marker_clear_since) >= CAR_MARKER_CLEAR_MS)
        {
            car->lap_armed = true;
        }
    }
    else
    {
        car->marker_clear_pending = false;
    }

    if (entered && car->lap_armed)
    {
        /* 太早的横线事件也消费本次许可，不能靠停留到超时再补计一圈。 */
        car->lap_armed = false;
        car->marker_clear_pending = false;
        if ((uint32_t)(now - car->lap_started_at) >= CAR_MIN_LAP_TIME_MS)
        {
            ++car->completed_laps;
            car->lap_started_at = now;
            if (car->completed_laps == CAR_TARGET_LAPS)
            {
                Stop(car, CAR_DONE);
            }
        }
    }
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
    car->marker_changed_at = now_ms;
}

void CarControl_Update(CarControl *car, uint32_t now_ms,
                       uint8_t black_mask, bool pressed,
                       bool sensor_valid, bool enabled)
{
    static const int16_t weights[CAR_SENSOR_COUNT] = {-200, -143, -86, -29, 29, 86, 143, 200};
    int32_t sum = 0;
    int32_t count = 0;
    int32_t error;
    int32_t derivative;
    int32_t correction;
    int32_t magnitude;
    int32_t base;
    bool marker_entered;
    bool button_event = ButtonPressedEvent(car, now_ms, pressed);

    if (!enabled || !sensor_valid)
    {
        Stop(car, enabled ? CAR_FAULT_SENSOR : CAR_STOPPED);
        car->marker_raw = car->marker_stable = false;
        car->marker_changed_at = now_ms;
        return;
    }
    marker_entered = MarkerEntered(car, now_ms, black_mask);

    if (button_event)
    {
        if (car->state == CAR_RUNNING)
        {
            Stop(car, CAR_STOPPED);
        }
        else
        {
            /* 必须从横向起终点线启动，否则首个横线无法代表完整一圈。 */
            if (CAR_TARGET_LAPS > 0U &&
                (!car->marker_stable || black_mask != CAR_MARKER_MASK))
            {
                Stop(car, CAR_START_REQUIRED);
                return;
            }
            /* 开始新一次运行时清除上次的偏差和丢线计时。 */
            car->state = CAR_RUNNING;
            car->previous_error = 0;
            car->previous_error_valid = false;
            car->last_direction = 0;
            car->line_missing = false;
            car->completed_laps = 0U;
            car->lap_armed = false;
            car->marker_clear_pending = false;
            car->lap_started_at = now_ms;
            car->last_control_at = now_ms - CAR_CONTROL_PERIOD_MS;
            /* 当前起点事件已用于启动，不计圈。 */
            marker_entered = false;
        }
    }

    if (CAR_TARGET_LAPS > 0U && car->state == CAR_RUNNING)
    {
        /* 每次输入采样都检查横线，计圈不受 5 ms 转向周期限制。 */
        UpdateLaps(car, now_ms, black_mask, marker_entered);
    }
    if (car->state != CAR_RUNNING ||
        (uint32_t)(now_ms - car->last_control_at) < CAR_CONTROL_PERIOD_MS)
    {
        return;
    }
    car->last_control_at = now_ms;

    if (black_mask == 0U)
    {
        if (!car->line_missing)
        {
            car->line_missing = true;
            car->line_missing_at = now_ms;
        }
        if ((uint32_t)(now_ms - car->line_missing_at) >= CAR_LINE_LOST_TIMEOUT_MS)
        {
            /* 丢线过久锁定停车，放回起终点横线后按键开启新任务。 */
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

    /* 全黑表示横线：首次启动/未达目标圈数时低速通过，完成时已停车返回。 */
    if (black_mask == CAR_MARKER_MASK)
    {
        car->left_pwm = CAR_WIDE_LINE_SPEED;
        car->right_pwm = CAR_WIDE_LINE_SPEED;
        car->previous_error_valid = false;
        return;
    }

    for (uint8_t i = 0U; i < CAR_SENSOR_COUNT; ++i)
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
