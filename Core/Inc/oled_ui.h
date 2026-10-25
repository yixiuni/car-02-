#ifndef OLED_UI_H
#define OLED_UI_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint8_t task, laps, completed, state, fault, phase;
    bool ready, active;
    uint32_t elapsed_ms;
} OledView;
void OledUI_Lines(const OledView *view, char lines[8][22]);
void OledUI_Render(const OledView *view, uint8_t frame[1024]);
#endif
