#pragma once

#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 0
#define FW_VERSION_PATCH 0
#define _FW_STR(x)  #x
#define _FW_XSTR(x) _FW_STR(x)
#define FW_VERSION  _FW_XSTR(FW_VERSION_MAJOR) "." _FW_XSTR(FW_VERSION_MINOR) "." _FW_XSTR(FW_VERSION_PATCH)
#define FW_BUILD_TIMESTAMP (__DATE__ " " __TIME__)

#include <stdint.h>

// ============================================================
// Hardware Configuration — Hiwonder MechDog (ESP32-WROOM-32D / D0WD chip)
// ============================================================
// Main controller: ESP32-WROOM-32D module (D0WD chip). The separate
// ESP32-S3 on the vision module is for the camera only and is
// not connected in the current configuration.
// All pin assignments verified via stock firmware REPL.
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
// Hardware LEDC peripheral: servo index i → LEDC channel i, 50Hz, 14-bit.
// Order: FL_hip, FL_knee, FR_hip, FR_knee, RL_hip, RL_knee, RR_hip, RR_knee
static const uint8_t SERVO_PINS[8] = {
    25, 26, 27, 14, 16, 17, 4, 2
};

// --- Servo Parameters ---
#define SERVO_FREQ_HZ       50
#define SERVO_MIN_US        500
#define SERVO_MAX_US        2500
#define SERVO_CENTER_US     1500

// --- LEDC Hardware PWM ---
#define LEDC_RESOLUTION     14                              // 14-bit: 16384 ticks per 20ms period
#define LEDC_MAX_DUTY       ((1 << LEDC_RESOLUTION) - 1)  // 16383

// --- Servo Soft-Start ---
#define SOFTSTART_DURATION_MS   2000    // ramp from lying-down to standing over 2s

// --- Shutdown Timing ---
#define SHUTDOWN_RAMP_MS    1500    // ramp to rest pose on disengage (non-blocking, millis-based)

// --- Engage/Disengage Timing ---
#define REST_SETTLE_MS      500  // pause at rest pose before detaching servos (used by lifecycle)

// --- Standing Pose (servo pulse widths in μs) ---
// Captured from stock firmware set_default_pose() + offsets
// Order: FL_hip, FL_knee, FR_hip, FR_knee, RL_hip, RL_knee, RR_hip, RR_knee
// Verified from stock firmware read_all_servo() at default pose
static const uint16_t STANDING_POSE[8] = {
    2096, 1621, 2170, 1611, 904, 1379, 830, 1389
};

// --- Lying-Down Pose (servo pulse widths in μs) ---
// Center (1500us) — safe neutral position used on boot and shutdown.
// Order: FL_hip, FL_knee, FR_hip, FR_knee, RL_hip, RL_knee, RR_hip, RR_knee
static const uint16_t LYING_DOWN_POSE[8] = {
    1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500
};

// --- Rest Pose (servo pulse widths in μs) ---
// Resting pose: hips back, knees partially tucked. Used for engage/disengage transitions.
// Order: FL_hip, FL_knee, FR_hip, FR_knee, RL_hip, RL_knee, RR_hip, RR_knee
static const uint16_t REST_POSE[8] = {
    1800, 1500, 1870, 1500, 1202, 1500, 1165, 1500
};

// --- Battery ADC (VERIFIED: Hiwonder.__adcp = ADC(Pin(34), atten=3)) ---
#define BATTERY_ADC_PIN     34
#define BATTERY_DIVIDER     3.9f    // voltage divider ratio (~1:4 resistor divider)
#define BATTERY_LOW_MV      6400    // 2S LiPo cutoff (~3.2V/cell)

// --- WiFi ---
// Credentials come from config_local.h (gitignored).
// Copy firmware/include/config_local.h.example → config_local.h and fill in values.
#if __has_include("config_local.h")
#include "config_local.h"
#endif
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
#define HEARTBEAT_TIMEOUT_MS 10000

// --- Gait Parameters ---
#define GAIT_STRIDE_HEIGHT_MM   10.0f   // foot lift height mm (conservative for first hardware tests)
#define GAIT_STRIDE_LENGTH_MM   12.0f   // forward/back foot swing mm (conservative)
#define GAIT_FREQUENCY_HZ       1.5f    // steps per second

// --- Return-to-stand taper ---
// When gait transitions back to STAND/STOP, servo positions are interpolated
// from wherever they are to the target standing pose over this duration.
// Prevents violent snap-to-standing on control release.
#define STAND_RETURN_MS         600     // ms to blend from gait position to standing

// --- Servo polarity overrides ---
// The IK auto-derives polarity from standing pulse: >1500 → +1, <1500 → -1.
// If a servo is physically mounted in reverse, override with +1 or -1 here.
// 0 = use auto-derived polarity.
// Hypothesis: rear legs (RL/RR, indices 4-7) may need polarity flipped to +1.
// Test: in test mode, send index 4 pulse 800 — if RL hip swings forward, polarity=-1 is correct.
//       If it swings backward, set polarity_override[4]=+1.
// clang-format off
#ifndef IK_POLARITY_OVERRIDE_DEFINED
#define IK_POLARITY_OVERRIDE_DEFINED
static const int8_t SERVO_POLARITY_OVERRIDE[8] = {
    0,   // 0 FL_hip  — auto (+1, standing=2096)
    0,   // 1 FL_knee — auto (+1, standing=1621)
    0,   // 2 FR_hip  — auto (+1, standing=2170)
    0,   // 3 FR_knee — auto (+1, standing=1611)
    0,   // 4 RL_hip  — auto (-1, standing=904)  VERIFIED: pulse↓ → forward swing ✓
    0,   // 5 RL_knee — auto (-1, standing=1379)
   +1,   // 6 RR_hip  — OVERRIDE: pulse↓ → backward (inverted); +1 corrects it
   -1,   // 7 RR_knee — OVERRIDE: physically inverted with RR_hip; -1 corrects it
};
#endif
// clang-format on

// --- User Button ---
// K1 SW-PB_3x6x4.3, GPIO 5, active-LOW, 10K pullup to 3V3. Confirmed V1.2 schematic.
#define BUTTON_PIN                  5
#define BUTTON_DEBOUNCE_MS          20
#define BUTTON_LONG_PRESS_MS        1000
// Set to 1 to toggle engage on short press when host has no lock.
#define BUTTON_LOCAL_ENGAGE_TOGGLE  0

// --- IMU Interrupt ---
// QMI8658 INT2 (data-ready) wired to GPIO 35 (input-only). Confirmed V1.2 schematic.
#define IMU_INT_PIN              35
#define IMU_INT_SLACK_MS         30   // ISR wake slack for safety timeout

// --- Onboard Blue LED ---
// GPIO 18, active-LOW. max(r,g,b)>0 = on via cmd_led {led:0,...}. Confirmed V1.2 schematic.
#define ONBOARD_LED_PIN     18

// --- Buzzer ---
// GPIO 21, driven via S8050 NPN transistor. HIGH = buzzer on. Confirmed V1.2 schematic.
#define BUZZER_PIN          21
#define BUZZER_LEDC_CH      9   // LEDC channel — 0-7 leg servos, 8 probe, 9 buzzer

// --- LED Brightness ---
// Scale is 0-255. Stock firmware uses full 0-255 range.
#define LED_BRIGHTNESS  40  // dim but clearly visible at ~16% max

// --- LED Colors (0-255 scale) ---
// Lavender: R=180, G=110, B=255 at full brightness, scaled to LED_BRIGHTNESS
#define LED_R_LAVENDER  ((180 * LED_BRIGHTNESS) / 255)  // ~28
#define LED_G_LAVENDER  ((110 * LED_BRIGHTNESS) / 255)  // ~17
#define LED_B_LAVENDER  LED_BRIGHTNESS                   // 40
