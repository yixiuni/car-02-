#ifndef GRAY_PROTOCOL_H
#define GRAY_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool waiting, candidate, valid;
    uint8_t raw_status;
    uint32_t request_at, received_at, activity_at, last_valid_at;
    uint32_t samples, timeouts, errors, unexpected_bytes;
} GrayProtocol;

void GrayProtocol_Init(volatile GrayProtocol *gray, uint32_t now);
bool GrayProtocol_CanRequest(const volatile GrayProtocol *gray, uint32_t now);
void GrayProtocol_BeginRequest(volatile GrayProtocol *gray, uint32_t now);
void GrayProtocol_OnByte(volatile GrayProtocol *gray, uint32_t now,
                          uint8_t byte, bool uart_error);
void GrayProtocol_Poll(volatile GrayProtocol *gray, uint32_t now);
bool GrayProtocol_Online(const volatile GrayProtocol *gray, uint32_t now);
uint8_t GrayProtocol_BlackMask(uint8_t raw_status);

#endif
