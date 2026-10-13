#include "gray_protocol.h"
#include "car_config.h"

void GrayProtocol_Init(volatile GrayProtocol *gray, uint32_t now)
{
    *gray = (GrayProtocol){0};
    gray->activity_at = now;
}

bool GrayProtocol_CanRequest(const volatile GrayProtocol *gray, uint32_t now)
{
    return !gray->waiting &&
           (uint32_t)(now - gray->activity_at) >= CAR_GRAY_QUERY_PERIOD_MS;
}

void GrayProtocol_BeginRequest(volatile GrayProtocol *gray, uint32_t now)
{
    gray->request_at = now;
    gray->waiting = true;
    gray->candidate = false;
}

void GrayProtocol_OnByte(volatile GrayProtocol *gray, uint32_t now,
                          uint8_t byte, bool uart_error)
{
    gray->activity_at = now;
    if (uart_error)
    {
        ++gray->errors;
        gray->valid = gray->waiting = gray->candidate = false;
        return;
    }
    if (!gray->waiting ||
        (uint32_t)(now - gray->request_at) >= CAR_GRAY_RESPONSE_TIMEOUT_MS)
    {
        ++gray->unexpected_bytes;
        gray->valid = gray->waiting = gray->candidate = false;
        return;
    }
    if (gray->candidate)
    {
        /* State replies are exactly one byte, not 55 AA analog/threshold frames. */
        ++gray->errors;
        gray->valid = gray->waiting = gray->candidate = false;
        return;
    }
    gray->raw_status = byte;
    gray->received_at = now;
    gray->candidate = true;
}

void GrayProtocol_Poll(volatile GrayProtocol *gray, uint32_t now)
{
    if (gray->waiting && gray->candidate &&
        (uint32_t)(now - gray->received_at) >= CAR_GRAY_SETTLE_MS)
    {
        gray->valid = true;
        gray->waiting = gray->candidate = false;
        gray->last_valid_at = gray->received_at;
        ++gray->samples;
    }
    else if (gray->waiting &&
             (uint32_t)(now - gray->request_at) >= CAR_GRAY_RESPONSE_TIMEOUT_MS)
    {
        gray->valid = gray->waiting = gray->candidate = false;
        gray->activity_at = now; /* quiet interval before retry; reject late bytes */
        ++gray->timeouts;
    }
}

bool GrayProtocol_Online(const volatile GrayProtocol *gray, uint32_t now)
{
    return gray->valid &&
           (uint32_t)(now - gray->last_valid_at) < CAR_GRAY_STALE_TIMEOUT_MS;
}

uint8_t GrayProtocol_BlackMask(uint8_t raw_status)
{
    uint8_t mask = CAR_GRAY_TARGET_BIT_IS_ONE ? raw_status : (uint8_t)~raw_status;
    if (CAR_GRAY_REVERSE_ORDER)
    {
        uint8_t reversed = 0U;
        for (uint8_t i = 0U; i < 8U; ++i)
        {
            reversed |= (uint8_t)(((mask >> i) & 1U) << (7U - i));
        }
        mask = reversed;
    }
    return mask;
}
