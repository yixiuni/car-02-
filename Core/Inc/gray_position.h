#ifndef GRAY_POSITION_H
#define GRAY_POSITION_H
#include <stdbool.h>
#include <stdint.h>

/* Experimental position estimate. It never confirms a course point and must
 * not replace the existing fresh-frame/footprint gates. Calibrate at the same
 * sensor height; white/black references alone do not prove spatial accuracy. */
typedef struct {
    float white[8],black[8];
    float pitch_mm;
    bool reversed;
} GrayPositionCalibration;
typedef enum {
    GRAY_POSITION_OK, GRAY_POSITION_BAD_CALIBRATION, GRAY_POSITION_WHITE,
    GRAY_POSITION_WIDE, GRAY_POSITION_SPLIT
} GrayPositionStatus;
typedef struct {
    float position_mm,strength,peak,spread_mm2;
    float weights[8];
    uint8_t active_mask,clipped_mask;
    GrayPositionStatus status;
} GrayPosition;
bool GrayPosition_Estimate(const uint16_t raw[8],
    const GrayPositionCalibration *cal,GrayPosition *out);
#endif
