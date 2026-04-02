#include "sonar.h"
#include "config.h"

// I2C sonar registers
#define SONAR_REG_DISTANCE  0x00
#define SONAR_REG_RGB_MODE  0x04
#define SONAR_REG_RGB1_R    0x05
#define SONAR_REG_RGB1_G    0x06
#define SONAR_REG_RGB1_B    0x07
#define SONAR_REG_RGB2_R    0x08
#define SONAR_REG_RGB2_G    0x09
#define SONAR_REG_RGB2_B    0x0A

static TwoWire* _wire = nullptr;

static bool writeReg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(SONAR_ADDR);
    _wire->write(reg);
    _wire->write(val);
    return _wire->endTransmission() == 0;
}

static bool readReg(uint8_t reg, uint8_t* buf, uint8_t len) {
    _wire->beginTransmission(SONAR_ADDR);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    if (_wire->requestFrom((uint8_t)SONAR_ADDR, len) != len) return false;
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = _wire->read();
    }
    return true;
}

bool sonar_init(TwoWire& wire) {
    _wire = &wire;

    // Set RGB mode to manual
    sonar_set_rgb_mode(0);

    // Set both LEDs to dim blue
    sonar_set_rgb(1, 0, 0, LED_BRIGHTNESS);
    sonar_set_rgb(2, 0, 0, LED_BRIGHTNESS);

    return true;
}

uint16_t sonar_read_mm() {
    if (!_wire) return 0;

    uint8_t buf[2];
    if (!readReg(SONAR_REG_DISTANCE, buf, 2)) return 0;

    // Big-endian: high byte first
    return (uint16_t)(buf[0] << 8) | buf[1];
}

void sonar_set_rgb_mode(uint8_t mode) {
    if (!_wire) return;
    writeReg(SONAR_REG_RGB_MODE, mode);
}

void sonar_set_rgb(uint8_t led, uint8_t r, uint8_t g, uint8_t b) {
    if (!_wire) return;

    uint8_t base;
    if (led == 1) {
        base = SONAR_REG_RGB1_R;
    } else if (led == 2) {
        base = SONAR_REG_RGB2_R;
    } else {
        return;
    }

    writeReg(base,     r);
    writeReg(base + 1, g);
    writeReg(base + 2, b);
}
