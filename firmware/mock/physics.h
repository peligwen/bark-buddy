#pragma once
#include <cstdint>

namespace physics {

struct IMUSample {
    float pitch_deg, roll_deg, yaw_deg;
    float ax, ay, az;   // body-frame accelerometer (g)
    float gx, gy, gz;   // body-frame gyro (deg/s)
};

void init();
void on_servo_duty(uint8_t pin, uint32_t duty);
void tick(float dt_s);

void imu_sample(IMUSample& out);
uint16_t sonar_mm();
int battery_raw();

}  // namespace physics
