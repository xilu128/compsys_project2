
#ifndef P2_PACKET_H
#define P2_PACKET_H
#include <stdint.h>
#include <math.h>

static inline int32_t P2_HeadingDisplay(float degrees, int valid, unsigned status) {
    if (!valid) return -32768 + (status >= 1U && status <= 4U ? (int32_t)status-1 : 0);
    if (!isfinite(degrees)) return -32764;
    float wrapped = fmodf(degrees, 360.0f);
    if (wrapped < 0.0f) wrapped += 360.0f;
    return (int32_t)lroundf(wrapped) % 360;
}

static inline int32_t P2_HeadingWire(float degrees, int valid, unsigned status) {
    int32_t display = P2_HeadingDisplay(degrees, valid, status);
    return display >= 0 ? display * 10 : display;
}
static inline void P2_PutI16(uint8_t *p, int32_t value) {
    if (value > 32767) value = 32767;
    if (value < -32768) value = -32768;
    uint16_t u = (uint16_t)value;
    p[0] = (uint8_t)u;
    p[1] = (uint8_t)(u >> 8);
}
#endif
