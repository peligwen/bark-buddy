#pragma once
#include <cstdint>
#include <cstddef>
// TwoWire no-op — IMU and sonar are intercepted at the driver level via link substitution.
struct TwoWire {
    void begin(int = -1, int = -1) {}
    void beginTransmission(uint8_t) {}
    uint8_t endTransmission(bool = true) { return 0; }
    uint8_t requestFrom(uint8_t, uint8_t) { return 0; }
    int available() { return 0; }
    int read() { return 0; }
    size_t write(uint8_t) { return 1; }
    size_t write(const uint8_t*, size_t n) { return n; }
};
extern TwoWire Wire;
