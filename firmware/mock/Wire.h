#pragma once
#include <cstdint>
#include <cstddef>
// TwoWire no-op — IMU and sonar are intercepted at the driver level via link substitution.
// Wire1 (bus 2) also resolves to a no-op stub; mock scan always returns empty (correct — no
// devices are wired to J4 on the dev machine).
struct TwoWire {
    void begin(int = -1, int = -1, uint32_t = 0) {}
    void beginTransmission(uint8_t) {}
    // Returns non-zero by default so bus scans find no devices (correct for mock/Wire1).
    uint8_t endTransmission(bool = true) { return 1; }
    uint8_t requestFrom(int, int) { return 0; }
    uint8_t requestFrom(uint8_t, uint8_t) { return 0; }
    int available() { return 0; }
    int read() { return 0; }
    size_t write(uint8_t) { return 1; }
    size_t write(const uint8_t*, size_t n) { return n; }
};
extern TwoWire Wire;
extern TwoWire Wire1;
