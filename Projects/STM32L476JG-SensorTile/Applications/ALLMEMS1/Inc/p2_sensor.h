#ifndef P2_SENSOR_H
#define P2_SENSOR_H
#include <stdint.h>
#include <stdbool.h>
#include "p2_motion.h"
typedef int32_t (*P2_RegisterIO)(uint16_t reg, uint8_t *data, uint16_t length);
typedef struct { P2_RegisterIO read_acc, write_acc, read_mag, write_mag; } P2_SensorBus;
bool P2_SensorInit(const P2_SensorBus *bus);
/* 1 = new sample; 0 = not ready; -1 = bus error. Raw physical units, no calibration. */
int P2_SensorReadAcc(const P2_SensorBus *bus, P2_Vector *sample);
int P2_SensorReadMag(const P2_SensorBus *bus, P2_Vector *sample);
/* Optional LSM6DSM gyro, independent of the required LSM303AGR sensors.
 * Callbacks address the LSM6DSM primary SPI interface (SensorTile CS = PB12).
 * Read uses reg | 0x80, NOT the LSM303AGR accelerometer's 0x40 burst bit;
 * CTRL3_C.IF_INC supplies LSM6DSM auto-increment. */
typedef struct { P2_RegisterIO read, write; } P2_GyroBus;
/* 104 Hz, +/-2000 dps; sensitivity 0.07 degrees/second per signed count. */
bool P2_GyroInit(const P2_GyroBus *bus);
/* Native LSM6DSM register-axis XYZ in degrees/second, no bias or board remap.
 * 1 = new sample; 0 = not ready; -1 = bus/argument error. On 0/-1, sample is
 * unchanged. Init/read retries and startup settling are owned by the caller. */
int P2_GyroRead(const P2_GyroBus *bus, P2_Vector *sample);
#endif
