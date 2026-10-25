#ifndef HIWONDER_GRAY_H
#define HIWONDER_GRAY_H

#include "gray_protocol.h"

void HiwonderGray_Init(uint32_t now);
void HiwonderGray_Poll(uint32_t now);
void HiwonderGray_IRQHandler(void);
void HiwonderGray_Snapshot(GrayProtocol *snapshot);

#endif
