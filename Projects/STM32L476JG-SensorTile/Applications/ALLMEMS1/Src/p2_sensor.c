#include "p2_sensor.h"
static bool configure(P2_RegisterIO read, P2_RegisterIO write, uint16_t reg, uint8_t value) {
    uint8_t actual = 0;
    return write(reg, &value, 1) == 0 && read(reg, &actual, 1) == 0 && actual == value;
}
bool P2_SensorInit(const P2_SensorBus *b) {
    uint8_t acc_id = 0, mag_id = 0;
    if (b->read_acc(0x0F, &acc_id, 1) || b->read_mag(0x4F, &mag_id, 1) ||
        acc_id != 0x33 || mag_id != 0x40) return false;
    /* LSM303AGR datasheet: BDU + +/-4g + high resolution + three-wire SPI.
     * 50 Hz, normal power, XYZ enabled. Sensitivity = 2 mg/12-bit count. */
    return configure(b->read_acc, b->write_acc, 0x23, 0x99) &&
           configure(b->read_acc, b->write_acc, 0x20, 0x47) &&
           configure(b->read_mag, b->write_mag, 0x62, 0x30) &&
           configure(b->read_mag, b->write_mag, 0x61, 0x03) &&
           configure(b->read_mag, b->write_mag, 0x60, 0x88);
    /* Mag: BDU + I2C_DIS; internal offset cancellation + LPF;
     * temperature compensation, 50 Hz, continuous high-resolution mode. */
}
static int32_t signed_word(const uint8_t *p) {
    uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8);
    return u >= 32768U ? (int32_t)u - 65536 : (int32_t)u;
}
static int read_axes(P2_RegisterIO read, uint16_t status, uint16_t output, float scale, P2_Vector *v) {
    uint8_t ready, data[6];
    if (read(status, &ready, 1)) return -1;
    if (!(ready & 0x08U)) return 0;
    if (read(output, data, 6)) return -1;
    v->x = signed_word(data) * scale;
    v->y = signed_word(data+2) * scale;
    v->z = signed_word(data+4) * scale;
    return 1;
}
int P2_SensorReadAcc(const P2_SensorBus *b, P2_Vector *v) {
    return read_axes(b->read_acc, 0x27, 0x28, 2.0f/16.0f, v);
}
int P2_SensorReadMag(const P2_SensorBus *b, P2_Vector *v) {
    return read_axes(b->read_mag, 0x67, 0x68, 1.5f, v);
}

bool P2_GyroInit(const P2_GyroBus *b) {
    uint8_t id = 0;
    if (!b || !b->read || !b->write) return false;
    /* LSM6DSM datasheet, sections 6.5.3 and 10.12-10.19:
     * https://www.st.com/resource/en/datasheet/lsm6dsm.pdf
     * Writes work before SIM is set; reads on this board require three-wire.
     * Bootstrap SIM before WHO_AM_I so retry also works after sensor power loss.
     * BDU + SIM + IF_INC, little endian. Do not soft-reset back to four-wire. */
    if (!configure(b->read, b->write, 0x12, 0x4C) ||
        b->read(0x0F, &id, 1) || id != 0x6A) return false;
    /* Own only this optional gyro. Re-establish known settings on every retry:
     * power down while configuring, no FIFO/self-test/DEN/high-pass, then
     * 104 Hz high-performance, +/-2000 dps. The LSM303AGR is unchanged.
     * CTRL2_G: ODR_G=0100 (104 Hz), FS_G=11 (2000 dps), FS_125=0. */
    return configure(b->read, b->write, 0x11, 0x00) &&
           configure(b->read, b->write, 0x0A, 0x00) &&
           configure(b->read, b->write, 0x13, 0x04) &&
           configure(b->read, b->write, 0x14, 0x00) &&
           configure(b->read, b->write, 0x15, 0x00) &&
           configure(b->read, b->write, 0x16, 0x00) &&
           configure(b->read, b->write, 0x11, 0x4C);
}

int P2_GyroRead(const P2_GyroBus *b, P2_Vector *v) {
    uint8_t ready, data[6];
    if (!b || !b->read || !v) return -1;
    if (b->read(0x1E, &ready, 1)) return -1;
    if (!(ready & 0x02U)) return 0; /* GDA, not XLDA or temperature-ready. */
    if (b->read(0x22, data, sizeof data)) return -1;
    /* +/-2000 dps sensitivity = 70 mdps/LSB = 0.07 dps/LSB.
     * BDU and one six-byte burst prevent torn low/high words. */
    v->x = signed_word(data) * 0.07f;
    v->y = signed_word(data + 2) * 0.07f;
    v->z = signed_word(data + 4) * 0.07f;
    return 1;
}
