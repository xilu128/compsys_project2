#include "p2_sensor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint8_t registers[256];
static int fail_call, corrupt_register, calls, writes, axis_reads;

static int32_t read_register(uint16_t reg, uint8_t *data, uint16_t length) {
    ++calls;
    if (calls == fail_call) return -1;
    assert((registers[0x12] & 0x08U) != 0);
    assert(reg + length <= sizeof registers);
    if (reg == 0x22) {
        assert(length == 6);
        assert((registers[0x12] & 0x44U) == 0x44U);
        ++axis_reads;
    }
    memcpy(data, registers + reg, length);
    if ((int)reg == corrupt_register) data[0] ^= 1U;
    return 0;
}

static int32_t write_register(uint16_t reg, uint8_t *data, uint16_t length) {
    ++calls;
    if (calls == fail_call) return -1;
    assert(length == 1 && reg + length <= sizeof registers);
    if (writes == 0) assert(reg == 0x12 && data[0] == 0x4C);
    ++writes;
    memcpy(registers + reg, data, length);
    return 0;
}

static void reset_mock(void) {
    memset(registers, 0, sizeof registers);
    registers[0x0F] = 0x6A;
    fail_call = corrupt_register = -1;
    calls = writes = axis_reads = 0;
}

static void raw_word(unsigned reg, int value) {
    uint16_t u = (uint16_t)value;
    registers[reg] = (uint8_t)u;
    registers[reg + 1] = (uint8_t)(u >> 8);
}

static void unchanged(P2_Vector v) {
    assert(v.x == 11 && v.y == 22 && v.z == 33);
}

int main(void) {
    const P2_GyroBus bus = {read_register, write_register};
    P2_Vector value = {11, 22, 33};
    reset_mock();
    assert(P2_GyroInit(&bus));
    assert(registers[0x12] == 0x4C && registers[0x13] == 0x04);
    assert(registers[0x11] == 0x4C && registers[0x16] == 0x00);
    const int init_calls = calls;
    for (int failed = 1; failed <= init_calls; ++failed) {
        reset_mock(); fail_call = failed;
        assert(!P2_GyroInit(&bus));

        fail_call = -1;
        assert(P2_GyroInit(&bus));
    }
    for (unsigned i = 0; i < 7; ++i) {
        static const int checked[] = {0x12, 0x11, 0x0A, 0x13, 0x14, 0x15, 0x16};
        reset_mock(); corrupt_register = checked[i];
        assert(!P2_GyroInit(&bus));
    }
    reset_mock(); registers[0x0F] = 0x69;
    assert(!P2_GyroInit(&bus));
    reset_mock(); assert(P2_GyroInit(&bus));
    registers[0x1E] = 0;
    assert(P2_GyroRead(&bus, &value) == 0); unchanged(value);
    registers[0x1E] = 0x05;
    assert(P2_GyroRead(&bus, &value) == 0); unchanged(value);
    assert(axis_reads == 0);
    registers[0x1E] = 0x02;
    fail_call = calls + 1;
    assert(P2_GyroRead(&bus, &value) == -1); unchanged(value);
    fail_call = calls + 2;
    assert(P2_GyroRead(&bus, &value) == -1); unchanged(value);
    fail_call = -1;
    raw_word(0x22, 1000); raw_word(0x24, -1000); raw_word(0x26, -32768);
    assert(P2_GyroRead(&bus, &value) == 1);
    assert(fabsf(value.x - 70.0f) < 0.0001f);
    assert(fabsf(value.y + 70.0f) < 0.0001f);
    assert(fabsf(value.z + 2293.76f) < 0.001f);
    raw_word(0x22, 32767); raw_word(0x24, -1); raw_word(0x26, 0);
    assert(P2_GyroRead(&bus, &value) == 1);
    assert(fabsf(value.x - 2293.69f) < 0.001f);
    assert(fabsf(value.y + 0.07f) < 0.0001f && value.z == 0);

    raw_word(0x22, 7143); raw_word(0x24, 10000); raw_word(0x26, 28571);
    assert(P2_GyroRead(&bus, &value) == 1);
    assert(fabsf(value.x - 500.01f) < 0.001f);
    assert(fabsf(value.y - 700.0f) < 0.001f);
    assert(fabsf(value.z - 1999.97f) < 0.001f);
    raw_word(0x22, -7143); raw_word(0x24, -10000); raw_word(0x26, -28571);
    assert(P2_GyroRead(&bus, &value) == 1);
    assert(fabsf(value.x + 500.01f) < 0.001f);
    assert(fabsf(value.y + 700.0f) < 0.001f);
    assert(fabsf(value.z + 1999.97f) < 0.001f);
    assert(!P2_GyroInit(NULL));
    assert(!P2_GyroInit(&(P2_GyroBus){NULL, write_register}));
    assert(!P2_GyroInit(&(P2_GyroBus){read_register, NULL}));
    assert(P2_GyroRead(NULL, &value) == -1);
    assert(P2_GyroRead(&(P2_GyroBus){NULL, write_register}, &value) == -1);
    assert(P2_GyroRead(&bus, NULL) == -1);
    puts("LSM6DSM gyro sensor tests passed");
    return 0;
}
