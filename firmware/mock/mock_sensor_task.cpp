// Mock sensor task — replaces firmware/src/sensor_task.cpp at link time.
// Runs 50 Hz IMU thread and 10 Hz sonar thread reading from the physics model.
#include "../include/sensor_task.h"
#include "physics.h"

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

bool sensor_i2c_write(uint8_t, uint8_t, uint8_t) { return true; }
