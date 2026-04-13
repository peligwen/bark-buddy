// firmware/include/sensor_task.h
#pragma once
#include <stdint.h>

// Latest values from IMU and sonar. Written by sensor task, read by main loop.
struct SensorSnapshot {
    float    pitch, roll, yaw;   // degrees (complementary filter)
    float    ax, ay, az;          // m/s²
    float    gx, gy, gz;          // deg/s
    uint16_t sonar_mm;
    bool     imu_ok;
    bool     sonar_ok;
};

struct LedCmd      { uint8_t led, r, g, b; };
struct I2cWriteCmd { uint8_t addr, reg, val; };

// Start the FreeRTOS sensor task. Initialises I2C, IMU, and sonar inside the
// task. Blocks until the first init pass completes (≤1s) so that imu_ok and
// sonar_ok in the snapshot are valid before setup() sends the boot message.
void sensor_task_start();

// Copy the latest sensor values into `out`. Thread-safe; takes snapshot mutex.
void sensor_snapshot_get(SensorSnapshot& out);

// Queue a LED colour change. Non-blocking — drops silently if the queue is
// full (depth 4). Safe to call from main loop or command handlers.
void sensor_led_set(uint8_t led, uint8_t r, uint8_t g, uint8_t b);

// Issue a raw I2C register write through the sensor task. Blocks until the
// task executes it (≤~10ms). Debug / probe only. Returns true if ACKed.
bool sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t val);
