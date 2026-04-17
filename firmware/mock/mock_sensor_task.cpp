// Mock sensor task — replaces firmware/src/sensor_task.cpp at link time.
// Runs a 50 Hz IMU thread and 10 Hz sonar thread writing directly to the snapshot
// under a mutex. sensor_imu_signal_ready() is defined here as a no-op for link
// compatibility — the mock has no FreeRTOS semaphore to signal.
#include "../include/sensor_task.h"
#include "../include/config.h"
#include "physics.h"
#include "Wire.h"

#include <mutex>
#include <thread>
#include <chrono>
#include <atomic>

static SensorSnapshot s_snapshot{};
static std::mutex     s_mutex;
static std::atomic<bool> s_running{false};

static void imu_thread() {
    using namespace std::chrono;
    auto next = steady_clock::now();
    while (s_running.load()) {
        physics::IMUSample s;
        physics::imu_sample(s);
        {
            std::lock_guard<std::mutex> lk(s_mutex);
            s_snapshot.pitch   = s.pitch_deg;
            s_snapshot.roll    = s.roll_deg;
            s_snapshot.yaw     = s.yaw_deg;
            s_snapshot.ax      = s.ax;
            s_snapshot.ay      = s.ay;
            s_snapshot.az      = s.az;
            s_snapshot.gx      = s.gx;
            s_snapshot.gy      = s.gy;
            s_snapshot.gz      = s.gz;
            s_snapshot.imu_ok  = true;
        }
        // Signal the sensor task that IMU data is ready. This drives the same
        // semaphore path used by the hardware INT2 ISR (imu_isr in sensor_task.cpp),
        // so the mock and real firmware wake up via identical task logic.
        sensor_imu_signal_ready();
        next += milliseconds(20);  // 50 Hz
        std::this_thread::sleep_until(next);
    }
}

static void sonar_thread() {
    using namespace std::chrono;
    auto next = steady_clock::now();
    while (s_running.load()) {
        uint16_t mm = physics::sonar_mm();
        {
            std::lock_guard<std::mutex> lk(s_mutex);
            s_snapshot.sonar_mm  = mm;
            s_snapshot.sonar_ok  = true;
        }
        next += milliseconds(100);  // 10 Hz
        std::this_thread::sleep_until(next);
    }
}

void sensor_task_start() {
    s_running.store(true);
    std::thread(imu_thread).detach();
    std::thread(sonar_thread).detach();
    // Brief settle so first snapshot is valid before setup() sends boot message.
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
}

void sensor_snapshot_get(SensorSnapshot& out) {
    std::lock_guard<std::mutex> lk(s_mutex);
    out = s_snapshot;
}

void sensor_led_set(uint8_t, uint8_t, uint8_t, uint8_t) {}

bool sensor_i2c_op(const I2cCmd& cmd, I2cResult& out) {
    // Mock: Wire/Wire1 are no-op stubs. Scan always returns empty (no real devices on dev
    // machine). Write returns ok=true (silent no-op). Read returns 0 bytes.
    out = I2cResult{};
    out.seq = cmd.seq;  // echo seq so callers can verify the result is not stale
    TwoWire& wire = (cmd.bus == 2) ? Wire1 : Wire;
    switch (cmd.op) {
        case I2cOp::SCAN:
            for (uint8_t a = 1; a < 127 && out.addr_count < 16; a++) {
                wire.beginTransmission(a);
                if (wire.endTransmission() == 0) out.addrs[out.addr_count++] = a;
            }
            out.ok = true;
            break;
        case I2cOp::READ:
            out.ok = false; // no real device
            break;
        case I2cOp::WRITE:
            out.ok = true;  // silent no-op
            break;
    }
    return true;
}

bool sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t val) {
    I2cCmd cmd = {};
    cmd.op   = I2cOp::WRITE;
    cmd.bus  = 1;
    cmd.addr = addr;
    cmd.reg  = reg;
    cmd.val  = val;
    I2cResult res;
    return sensor_i2c_op(cmd, res) && res.ok;
}

// No-op in the mock: the mock's sensor loop is driven by std::thread sleep_until,
// not a FreeRTOS semaphore. The function exists so that the imu_thread() call above
// compiles, and so that any code path that calls sensor_imu_signal_ready() links
// correctly in both mock and real-hardware builds.
void sensor_imu_signal_ready() {}

// GPIO subscription stubs — no real GPIO on the host. Subscribe/unsubscribe are
// no-ops; they return true/false consistent with allowlist checks only.
#include "../include/gpio_aux.h"
bool sensor_gpio_subscribe(uint8_t pin, GpioMode mode) {
    (void)mode;
    return gpio_aux_allowlisted(pin);
}
void sensor_gpio_unsubscribe(uint8_t pin) { (void)pin; }
