/** Existing course BLE packet, signed 16-bit fields, little endian. */
#ifndef P2_PACKET_H
#define P2_PACKET_H
#include <stdint.h>
#include <math.h>
/* Heading wire field is signed tenths, not the starter's whole-degree gyro
 * surrogate. Reserve out-of-range codes, never confuse -1 degree with invalid. */
static inline int32_t P2_HeadingWire(float degrees, int valid, unsigned status) {
    if (!valid) return -32768 + (status >= 1U && status <= 4U ? (int32_t)status-1 : 0);
    if (!isfinite(degrees) || degrees < -3200.0f || degrees > 3200.0f) return -32764;
    return (int32_t)lroundf(degrees * 10.0f);
}
static inline void P2_PutI16(uint8_t *p, int32_t value) {
    if (value > 32767) value = 32767;
    if (value < -32768) value = -32768;
    uint16_t u = (uint16_t)value;
    p[0] = (uint8_t)u;
    p[1] = (uint8_t)(u >> 8);
}
#endif
