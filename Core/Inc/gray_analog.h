#ifndef GRAY_ANALOG_H
#define GRAY_ANALOG_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
/* Hiwonder: 55 AA command length payload checksum; uint16 little endian.
 * Checksum is inverted sum of command, length and payload (no header). */
bool GrayAnalog_Decode(const uint8_t *bytes,size_t count,uint8_t command,uint16_t values[8]);
typedef struct {
    bool waiting,valid;
    uint8_t count,bytes[21];
    uint16_t values[8];
    uint32_t request_at,received_at,activity_at,last_valid_at;
    uint32_t samples,timeouts,errors,unexpected_bytes;
} GrayAnalogReceiver;
void GrayAnalog_Init(volatile GrayAnalogReceiver *r,uint32_t now);
bool GrayAnalog_CanRequest(const volatile GrayAnalogReceiver *r,uint32_t now);
void GrayAnalog_BeginRequest(volatile GrayAnalogReceiver *r,uint32_t now);
void GrayAnalog_OnByte(volatile GrayAnalogReceiver *r,uint32_t now,uint8_t byte,bool error);
void GrayAnalog_Poll(volatile GrayAnalogReceiver *r,uint32_t now);
uint8_t GrayAnalog_Classify(const uint16_t values[8],const uint16_t threshold[8],
    const uint16_t hysteresis[8],uint8_t black_high_mask,uint8_t previous);
#endif
