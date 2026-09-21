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
#endif
