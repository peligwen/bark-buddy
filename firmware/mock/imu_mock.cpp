// Mock IMU driver — reads from physics model instead of real I2C hardware.
// Replaces firmware/src/imu.cpp at link time.
#include "imu.h"
#include "Wire.h"
#include "physics.h"

bool imu_init(TwoWire&) { return true; }

bool imu_read(IMUData& out) {
    physics::IMUSample s;
    physics::imu_sample(s);
    out.pitch = s.pitch_deg;
    out.roll  = s.roll_deg;
    out.yaw   = s.yaw_deg;
    out.ax    = s.ax;
    out.ay    = s.ay;
    out.az    = s.az;
    out.gx    = s.gx;
    out.gy    = s.gy;
    out.gz    = s.gz;
    return true;
}
