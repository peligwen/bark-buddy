#include "imu.h"
#include "config.h"
#include <math.h>

// QMI8658 Registers
#define QMI_WHO_AM_I    0x00
#define QMI_REVISION    0x01
#define QMI_CTRL1       0x02
#define QMI_CTRL2       0x03
#define QMI_CTRL3       0x04
#define QMI_CTRL5       0x06
#define QMI_CTRL7       0x08
#define QMI_RESET       0x60
#define QMI_ACCEL_X_L   0x35
#define QMI_GYRO_X_L    0x3B

#define QMI_WHO_AM_I_VAL 0x05

// Accel: ±8g, ODR 128Hz → register value 0x65
#define QMI_CTRL2_VAL   0x65
// Gyro: ±512dps, ODR 128Hz → register value 0x54
#define QMI_CTRL3_VAL   0x54
// Low-pass filter enable for accel+gyro
#define QMI_CTRL5_VAL   0x11
// Enable accel + gyro
#define QMI_CTRL7_VAL   0x03

// Scale factors
#define ACCEL_SCALE     (9.81f / 4096.0f)   // ±8g → 4096 LSB/g
#define GYRO_SCALE      (1.0f / 64.0f)      // ±512dps → 64 LSB/dps

// Complementary filter coefficient: fraction of gyro vs accel.
// 0.95 = 95% gyro (fast response) + 5% accel (slow drift correction).
// At 50Hz the accel time constant is ~0.02/(1-0.95) = 0.4s.
#define CF_ALPHA        0.95f

static TwoWire* _wire = nullptr;
static unsigned long _lastTime = 0;
static float _yaw = 0.0f;
static float _cf_pitch = 0.0f;
static float _cf_roll  = 0.0f;
static bool  _cf_init  = false;

static bool writeReg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(QMI8658_ADDR);
    _wire->write(reg);
    _wire->write(val);
    return _wire->endTransmission() == 0;
}

static bool readReg(uint8_t reg, uint8_t* buf, uint8_t len) {
    _wire->beginTransmission(QMI8658_ADDR);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    if (_wire->requestFrom((uint8_t)QMI8658_ADDR, len) != len) return false;
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = _wire->read();
    }
    return true;
}

bool imu_init(TwoWire& wire) {
    _wire = &wire;

    // Verify WHO_AM_I
    uint8_t id = 0;
    if (!readReg(QMI_WHO_AM_I, &id, 1)) return false;
    if (id != QMI_WHO_AM_I_VAL) return false;

    // Soft reset
    writeReg(QMI_RESET, 0xB0);
    delay(30);

    // Configure interface: address auto-increment, big-endian disabled
    writeReg(QMI_CTRL1, 0x40);

    // Accel config: ±8g, ODR 128Hz
    writeReg(QMI_CTRL2, QMI_CTRL2_VAL);

    // Gyro config: ±512dps, ODR 128Hz
    writeReg(QMI_CTRL3, QMI_CTRL3_VAL);

    // Low-pass filter
    writeReg(QMI_CTRL5, QMI_CTRL5_VAL);

    // Enable accel + gyro
    writeReg(QMI_CTRL7, QMI_CTRL7_VAL);

    delay(30);

    _lastTime = millis();
    _yaw     = 0.0f;
    _cf_pitch = 0.0f;
    _cf_roll  = 0.0f;
    _cf_init  = false;

    return true;
}

bool imu_read(IMUData& out) {
    if (!_wire) return false;

    // Read 12 bytes: 6 accel + 6 gyro (contiguous from 0x35)
    uint8_t buf[12];
    if (!readReg(QMI_ACCEL_X_L, buf, 12)) return false;

    // Accel: int16 little-endian
    int16_t raw_ax = (int16_t)(buf[0]  | (buf[1]  << 8));
    int16_t raw_ay = (int16_t)(buf[2]  | (buf[3]  << 8));
    int16_t raw_az = (int16_t)(buf[4]  | (buf[5]  << 8));

    // Gyro: int16 little-endian
    int16_t raw_gx = (int16_t)(buf[6]  | (buf[7]  << 8));
    int16_t raw_gy = (int16_t)(buf[8]  | (buf[9]  << 8));
    int16_t raw_gz = (int16_t)(buf[10] | (buf[11] << 8));

    // Convert to physical units
    out.ax = raw_ax * ACCEL_SCALE;
    out.ay = raw_ay * ACCEL_SCALE;
    out.az = raw_az * ACCEL_SCALE;

    out.gx = raw_gx * GYRO_SCALE;
    out.gy = raw_gy * GYRO_SCALE;
    out.gz = raw_gz * GYRO_SCALE;

    // Accel-only pitch and roll (used as correction reference)
    float accel_pitch = atan2f(out.ax, sqrtf(out.ay * out.ay + out.az * out.az)) * 180.0f / (float)M_PI;
    float accel_roll  = atan2f(out.ay, sqrtf(out.ax * out.ax + out.az * out.az)) * 180.0f / (float)M_PI;

    unsigned long now = millis();
    float dt = (now - _lastTime) / 1000.0f;
    _lastTime = now;

    if (!_cf_init) {
        // Seed filter from accelerometer on first read
        _cf_pitch = accel_pitch;
        _cf_roll  = accel_roll;
        _cf_init  = true;
    } else if (dt > 0.0f && dt < 1.0f) {
        // Complementary filter: blend gyro integration with accel correction.
        // gy = pitch rate (rotation about Y), gx = roll rate (rotation about X).
        _cf_pitch = CF_ALPHA * (_cf_pitch + out.gy * dt) + (1.0f - CF_ALPHA) * accel_pitch;
        _cf_roll  = CF_ALPHA * (_cf_roll  + out.gx * dt) + (1.0f - CF_ALPHA) * accel_roll;

        // Yaw from gyro integration only (no magnetometer for correction)
        _yaw += out.gz * dt;
    }

    out.pitch = _cf_pitch;
    out.roll  = _cf_roll;
    out.yaw   = _yaw;

    return true;
}
