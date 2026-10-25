#include <assert.h>
#include <stdio.h>
#include "gray_protocol.h"
#include "car_config.h"

static void Reply(GrayProtocol *gray, uint32_t now, uint8_t status)
{
    GrayProtocol_BeginRequest(gray, now);
    GrayProtocol_OnByte(gray, now + 1U, status, false);
    GrayProtocol_Poll(gray, now + 1U + CAR_GRAY_SETTLE_MS);
}

int main(void)
{
    GrayProtocol gray;
    GrayProtocol_Init(&gray, 0U);
    assert(!GrayProtocol_Online(&gray, 0U));
    GrayProtocol_OnByte(&gray, 1U, 0xFFU, false);
    assert(gray.unexpected_bytes == 1U && !gray.valid);
    assert(!GrayProtocol_CanRequest(&gray, 1U));
    assert(GrayProtocol_CanRequest(&gray, 1U + CAR_GRAY_QUERY_PERIOD_MS));

    /* Every bit pattern, including zero and FF, is a valid single-byte reply. */
    for (unsigned mask = 0U; mask <= 255U; ++mask)
    {
        uint32_t now = 100U + mask * 50U;
        Reply(&gray, now, (uint8_t)mask);
        assert(gray.raw_status == mask && GrayProtocol_Online(&gray, now + 3U));
        uint8_t expected = CAR_GRAY_TARGET_BIT_IS_ONE ? (uint8_t)mask : (uint8_t)~mask;
        if (CAR_GRAY_REVERSE_ORDER)
        {
            unsigned reversed = 0U;
            for (unsigned bit = 0U; bit < 8U; ++bit)
                reversed |= ((expected >> bit) & 1U) << (7U - bit);
            expected = (uint8_t)reversed;
        }
        assert(GrayProtocol_BlackMask(gray.raw_status) == expected);
    }
    assert(gray.samples == 256U);
    assert(!GrayProtocol_Online(&gray, gray.last_valid_at + CAR_GRAY_STALE_TIMEOUT_MS));

    GrayProtocol_BeginRequest(&gray, 20000U);
    GrayProtocol_Poll(&gray, 20000U + CAR_GRAY_RESPONSE_TIMEOUT_MS);
    assert(gray.timeouts == 1U && !gray.valid && !gray.waiting);
    GrayProtocol_OnByte(&gray, 20025U, 0x18U, false);
    assert(!gray.valid); /* late reply must not renew freshness */

    GrayProtocol_BeginRequest(&gray, 21000U);
    GrayProtocol_OnByte(&gray, 21001U, 0x55U, false);
    assert(!gray.valid); /* not committed until the single-byte guard expires */
    GrayProtocol_OnByte(&gray, 21001U, 0xAAU, false);
    GrayProtocol_Poll(&gray, 21005U);
    assert(!gray.valid && gray.errors == 1U); /* analog frame rejected */

    Reply(&gray, 22000U, 0x18U);
    assert(gray.valid);
    GrayProtocol_BeginRequest(&gray, 22100U);
    GrayProtocol_OnByte(&gray, 22101U, 0xFFU, true);
    assert(!gray.valid && !gray.waiting && gray.errors == 2U);

    GrayProtocol_Init(&gray, UINT32_MAX - 20U);
    Reply(&gray, UINT32_MAX - 1U, 0x81U);
    assert(GrayProtocol_Online(&gray, 2U));
    assert(!GrayProtocol_Online(&gray, CAR_GRAY_STALE_TIMEOUT_MS));
    puts("PASS: UART status bytes, polarity/order, timeouts, late/unsolicited bytes, frame rejection, errors, tick wrap");
    return 0;
}
