// firmware/include/sensor_task.h
#pragma once
#include <stdint.h>
#include "gpio_aux.h"

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

enum class I2cOp : uint8_t { WRITE, READ, SCAN };

struct I2cCmd {
    I2cOp    op;
    uint8_t  bus;   // 1 or 2
    uint8_t  addr;
    uint8_t  reg;
    uint8_t  val;   // for WRITE
    uint8_t  len;   // for READ (max 32)
    uint16_t seq;   // sequence number; copied into I2cResult for stale-result detection
};

struct I2cResult {
    bool     ok;
    uint8_t  data[32];
    uint8_t  data_len;
    uint8_t  addrs[16]; // for SCAN
    uint8_t  addr_count;
    uint16_t seq;       // echoed from I2cCmd; mismatched seq means result is stale
};

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
// Convenience wrapper around sensor_i2c_op() for bus 1 WRITE operations.
bool sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t val);

// Submit an I2C operation (scan/read/write, bus 1 or 2) and wait for result.
// Blocks until the sensor task executes it (max ~300ms: 100ms queue + 200ms
// execute). Thread-safe. Returns false on timeout or queue error.
// CAUTION: SCAN stalls the sensor task ~130ms — avoid during active balance/gait.
bool sensor_i2c_op(const I2cCmd& cmd, I2cResult& out);

// Signal that IMU data is ready. Called from mock's 50 Hz IMU thread to drive
// the same semaphore path used by the hardware INT2 ISR. No-op on real hardware
// (the GPIO ISR fires instead). Safe to call from any thread context.
void sensor_imu_signal_ready();

// Subscribe a GPIO aux pin for 20 Hz polling. At most 4 simultaneous subscriptions.
// Returns false if the table is full or the pin is not allowlisted.
// Thread-safe (spinlock-protected on hardware, mutex on mock).
bool sensor_gpio_subscribe(uint8_t pin, GpioMode mode);

// Unsubscribe a GPIO aux pin. No-op if not currently subscribed.
// Thread-safe.
void sensor_gpio_unsubscribe(uint8_t pin);
