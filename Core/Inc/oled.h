#ifndef OLED_H
#define OLED_H
#include "oled_ui.h"
typedef struct {
    uint32_t frames, max_poll_cycles, max_control_interval_ms;
} OledDiagnostics;
extern volatile OledDiagnostics oled_diagnostics;
/* Call after Board_Init (JTAG disabled, SWD retained), before watchdog start. */
void Oled_Init(void);
/* At most 16 data bytes per call, about 0.65 ms including page commands. */
void Oled_Poll(uint32_t now, const OledView *view);
#endif
