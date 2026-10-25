#ifndef CAR_ARC_DEBUG_H
#define CAR_ARC_DEBUG_H
#include <stdint.h>
/* Diagnostic-only bits. They must never be consumed to change navigation. */
enum {
    ARC_EXIT_NO_LINE=1U, ARC_EXIT_WHITE_FRAMES=2U, ARC_EXIT_STALE=4U,
    ARC_EXIT_LATERAL=8U, ARC_EXIT_PROGRESS=16U, ARC_EXIT_TURN=32U,
    ARC_EXIT_HEADING=64U, ARC_EXIT_DISTANCE=128U,
    ARC_EXIT_CORRECTION=256U, ARC_EXIT_FOOTPRINT=512U
};
typedef struct {
    uint32_t tick, rejected;
    float x_mm,y_mm,heading_rad,progress_mm,turned_rad;
    float heading_error_rad,correction_mm,line_error_mm;
    uint32_t white_frames,line_age_ms;
} CarArcExitSample;
typedef struct {
    uint32_t point_events,black_samples,exit_attempts;
    float last_radial_error_mm,max_abs_radial_error_mm;
    CarArcExitSample first,last;
} CarArcDebug;
#define CAR_ARC_DEBUG_CAPACITY 8U
extern volatile CarArcDebug car_arc_debug[CAR_ARC_DEBUG_CAPACITY];
#endif
