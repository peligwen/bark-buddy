#pragma once
#include <Wire.h>

struct IMUData {
    float ax, ay, az;   // m/s²
    float gx, gy, gz;   // deg/s
    float pitch, roll;  // from accelerometer (degrees)
    float yaw;          // integrated from gyro (drifts)
};

bool imu_init(TwoWire& wire);
bool imu_read(IMUData& out);
