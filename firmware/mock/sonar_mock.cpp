// Mock sonar driver — reads from physics model instead of real I2C hardware.
// Replaces firmware/src/sonar.cpp at link time.
#include "sonar.h"
#include "Wire.h"
#include "physics.h"

bool sonar_init(TwoWire&) { return true; }

uint16_t sonar_read_mm() {
    return physics::sonar_mm();
}
