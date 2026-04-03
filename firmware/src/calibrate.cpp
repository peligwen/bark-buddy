#include "calibrate.h"
#include "config.h"
#include "servos.h"
#include "imu.h"
#include "protocol.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>

extern SemaphoreHandle_t i2c_mutex;
extern void send_json(const JsonDocument& doc);

static bool active = false;
static CalibrateSweep sweep;
static uint16_t current_step_us;
static unsigned long step_start;
static float baseline_pitch;
static float baseline_roll;

static void send_cal_point(uint16_t pulse_us, float pitch, float roll) {
    JsonDocument doc;
    doc["type"] = "telem_cal";
    doc["servo"] = sweep.servo;
    doc["pulse_us"] = pulse_us;
    doc["pitch"] = round(pitch * 100) / 100.0;
    doc["roll"] = round(roll * 100) / 100.0;
    // Include all servo positions
    JsonArray angles = doc["angles"].to<JsonArray>();
    for (int i = 0; i < 8; i++) {
        angles.add(servo_read_us(i));
    }
    send_json(doc);
}

static void return_to_standing() {
    for (int i = 0; i < 8; i++) {
        servo_write_us(i, STANDING_POSE[i]);
    }
}

void calibrate_start(const CalibrateSweep& s) {
    sweep = s;
    if (sweep.servo >= 8) return;
    if (sweep.step_us == 0) sweep.step_us = 10;
    if (sweep.dwell_ms == 0) sweep.dwell_ms = 300;
    if (sweep.tilt_limit <= 0) sweep.tilt_limit = 8.0f;

    // Hold all servos at standing, then read baseline IMU
    return_to_standing();
    delay(500);

    IMUData imu;
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50))) {
        imu_read(imu);
        xSemaphoreGive(i2c_mutex);
    }
    baseline_pitch = imu.pitch;
    baseline_roll = imu.roll;

    // Start sweep
    current_step_us = sweep.from_us;
    step_start = millis();
    servo_write_us(sweep.servo, current_step_us);
    active = true;
}

void calibrate_stop() {
    if (active) {
        return_to_standing();
        active = false;
    }
}

bool calibrate_update(unsigned long now_ms) {
    if (!active) return false;

    // Check if dwell time elapsed for current step
    if (now_ms - step_start < sweep.dwell_ms) return true;

    // Read IMU at end of dwell
    IMUData imu;
    bool imu_ok = false;
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(10))) {
        imu_ok = imu_read(imu);
        xSemaphoreGive(i2c_mutex);
    }

    if (imu_ok) {
        // Safety check
        if (fabsf(imu.pitch) > sweep.tilt_limit || fabsf(imu.roll) > sweep.tilt_limit) {
            // Abort — tilt exceeded
            return_to_standing();
            JsonDocument doc;
            doc["type"] = "cal_abort";
            doc["servo"] = sweep.servo;
            doc["reason"] = "tilt_limit";
            doc["pitch"] = round(imu.pitch * 10) / 10.0;
            doc["roll"] = round(imu.roll * 10) / 10.0;
            send_json(doc);
            active = false;
            return false;
        }

        send_cal_point(current_step_us, imu.pitch, imu.roll);
    }

    // Advance to next step
    if (sweep.from_us <= sweep.to_us) {
        current_step_us += sweep.step_us;
        if (current_step_us > sweep.to_us) {
            // Sweep complete
            return_to_standing();
            JsonDocument doc;
            doc["type"] = "cal_done";
            doc["servo"] = sweep.servo;
            send_json(doc);
            active = false;
            return false;
        }
    } else {
        current_step_us -= sweep.step_us;
        if (current_step_us < sweep.to_us) {
            return_to_standing();
            JsonDocument doc;
            doc["type"] = "cal_done";
            doc["servo"] = sweep.servo;
            send_json(doc);
            active = false;
            return false;
        }
    }

    servo_write_us(sweep.servo, current_step_us);
    step_start = now_ms;
    return true;
}

bool calibrate_active() {
    return active;
}
