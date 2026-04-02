#pragma once
#include <stdint.h>

// ============================================================
// Hardware Configuration — Hiwonder MechDog ESP32-S3
// ============================================================
// IMPORTANT: Pin assignments are PLACEHOLDERS until verified
// via stock firmware REPL introspection. Do NOT enable
// PINS_VERIFIED until you've confirmed each GPIO.
// ============================================================

#ifndef PINS_VERIFIED
#define PINS_VERIFIED 0
#endif

// --- I2C Bus ---
// Confirmed from stock MicroPython: IIC(1) → scl=23, sda=22
// NOTE: ESP32-S3 GPIO23/22 may differ from stock ESP32 numbering.
// These need verification on the actual Hiwonder board.
#define I2C_SDA_PIN     22
#define I2C_SCL_PIN     23
#define I2C_FREQ        400000  // 400kHz fast mode

// --- I2C Device Addresses ---
#define QMI8658_ADDR    0x6B    // QMI8658 IMU (alternate: 0x6A)
#define SONAR_ADDR      0x77    // Hiwonder I2C ultrasonic

// --- Servo PWM Pins (PLACEHOLDER — must verify!) ---
// 8 servos: 2 per leg (hip + knee)
// Order: FL_hip, FL_knee, FR_hip, FR_knee, RL_hip, RL_knee, RR_hip, RR_knee
static const uint8_t SERVO_PINS[8] = {
    4, 5, 6, 7, 15, 16, 17, 18  // PLACEHOLDER — DO NOT USE UNTIL VERIFIED
};

// --- Servo Parameters ---
#define SERVO_FREQ_HZ       50
#define SERVO_MIN_US        500
#define SERVO_MAX_US        2500
#define SERVO_CENTER_US     1500
#define SERVO_RESOLUTION    16      // LEDC resolution bits

// --- Servo Soft-Start ---
#define SOFTSTART_DURATION_MS   2000    // ramp from center to standing over 2s
#define SOFTSTART_STEPS         50      // interpolation steps

// --- Standing Pose (servo pulse widths in μs) ---
// PLACEHOLDER — capture from stock firmware
static const uint16_t STANDING_POSE[8] = {
    1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500
};

// --- Battery ADC ---
#define BATTERY_ADC_PIN     14      // PLACEHOLDER
#define BATTERY_DIVIDER     2.0f    // voltage divider ratio
#define BATTERY_LOW_MV      6400    // 2S LiPo cutoff (~3.2V/cell)
#define BATTERY_NOMINAL_MV  7400

// --- WiFi ---
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASS
#define WIFI_PASS ""
#endif
#define WIFI_TCP_PORT   9000

// --- Timing ---
#define TELEM_IMU_HZ        50
#define TELEM_SONAR_HZ      20
#define TELEM_BATTERY_HZ    1
#define TELEM_STATUS_HZ     1
#define GAIT_UPDATE_HZ      50
#define HEARTBEAT_TIMEOUT_MS 5000

// --- Gait Parameters ---
#define GAIT_HIP_AMPLITUDE   15.0f   // degrees
#define GAIT_KNEE_AMPLITUDE  10.0f   // degrees
#define GAIT_FREQUENCY       2.0f    // Hz (steps per second)
#define GAIT_PHASE_OFFSET    3.14159f // PI — diagonal pairs antiphase

// --- LED Brightness ---
#define LED_BRIGHTNESS  15  // 0-255, kept dim for subtle indicator
