/** Existing course BLE packet, signed 16-bit fields, little endian. */
#ifndef P2_PACKET_H
#define P2_PACKET_H
#include <stdint.h>
static inline void P2_PutI16(uint8_t *p, int32_t value) {
    if (value > 32767) value = 32767;
    if (value < -32768) value = -32768;
    uint16_t u = (uint16_t)value;
    p[0] = (uint8_t)u;
    p[1] = (uint8_t)(u >> 8);
}
#endif
