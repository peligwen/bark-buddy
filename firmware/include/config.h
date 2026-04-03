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
#define PINS_VERIFIED 1
#endif

// --- I2C Bus ---
// VERIFIED from stock MicroPython: Hiwonder_IIC.IIC(1) → I2C(0, scl=23, sda=22)
// I2C scan confirmed devices at 0x6A (QMI8658) and 0x77 (sonar)
#define I2C_SDA_PIN     22
#define I2C_SCL_PIN     23
#define I2C_FREQ        400000  // 400kHz fast mode

// --- I2C Device Addresses (VERIFIED via I2C scan) ---
#define QMI8658_ADDR    0x6A    // QMI8658 IMU (confirmed, not 0x6B)
#define SONAR_ADDR      0x77    // Hiwonder I2C ultrasonic

// --- Servo PWM Pins (ALL 8 VERIFIED via MCPWM register diff + IMU) ---
// MCPWM0: GPIOs 25,26,27,14,16,17 (confirmed via register scan)
// MCPWM1: GPIOs 4,2 (confirmed via MCPWM1 register diff + IMU)
// Custom firmware uses software PWM (works on all pins)
// Order: FL_hip, FL_knee, FR_hip, FR_knee, RL_hip, RL_knee, RR_hip, RR_knee
static const uint8_t SERVO_PINS[8] = {
    25, 26, 27, 14, 16, 17, 4, 2
};

// --- Servo Parameters ---
#define SERVO_FREQ_HZ       50
#define SERVO_MIN_US        500
#define SERVO_MAX_US        2500
#define SERVO_CENTER_US     1500

// --- Servo Soft-Start ---
#define SOFTSTART_DURATION_MS   2000    // ramp from center to standing over 2s
#define SOFTSTART_STEPS         50      // interpolation steps

// --- Standing Pose (servo pulse widths in μs) ---
// Captured from stock firmware set_default_pose() + offsets
// Order: FL_hip, FL_knee, FR_hip, FR_knee, RL_hip, RL_knee, RR_hip, RR_knee
static const uint16_t STANDING_POSE[8] = {
    2096, 1621, 2170, 1611, 904, 1379, 1389, 830
    // FL_hip FL_knee FR_hip FR_knee RL_hip RL_knee RR_hip RR_knee
};

// --- Battery ADC (VERIFIED: Hiwonder.__adcp = ADC(Pin(34), atten=3)) ---
#define BATTERY_ADC_PIN     34
#define BATTERY_DIVIDER     3.9f    // voltage divider ratio (~1:4 resistor divider)
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
#define GAIT_HIP_AMPLITUDE   8.0f    // degrees (conservative — tune up after testing)
#define GAIT_KNEE_AMPLITUDE  5.0f    // degrees
#define GAIT_FREQUENCY       1.5f    // Hz (steps per second)
#define GAIT_PHASE_OFFSET    3.14159f // PI — diagonal pairs antiphase

// --- Servo Idle ---
#define SERVO_IDLE_TIMEOUT_MS   30000   // detach servos after 30s no movement

// --- LED Brightness ---
#define LED_BRIGHTNESS  15  // 0-255, kept dim for subtle indicator
