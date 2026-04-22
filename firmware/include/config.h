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

// --- I2C Bus 1 ---
// VERIFIED from stock MicroPython: Hiwonder_IIC.IIC(1) → I2C(0, scl=23, sda=22)
// I2C scan confirmed devices at 0x6A (QMI8658) and 0x77 (sonar)
#define I2C_SDA_PIN     22
#define I2C_SCL_PIN     23
#define I2C_FREQ        400000  // 400kHz fast mode

// --- I2C Bus 2 ---
// SDA=19, SCL=13, J4 connector. Confirmed V1.2 schematic. Wire1 instance.
#define I2C2_SDA_PIN    19
#define I2C2_SCL_PIN    13
#define I2C2_FREQ       400000
#define I2C2_ENABLED    1

// --- I2C Device Addresses (VERIFIED via I2C scan) ---
#define QMI8658_ADDR    0x6A    // QMI8658 IMU (confirmed, not 0x6B)
#define SONAR_ADDR      0x77    // Hiwonder I2C ultrasonic

// --- Servo PWM Pins (ALL 8 VERIFIED via MCPWM register diff + IMU) ---
// MCPWM0: GPIOs 25,26,16,17,27,14 (confirmed via register scan)
// MCPWM1: GPIOs 15,2 (confirmed via MCPWM1 register diff + IMU)
// Hardware LEDC peripheral: servo index i → LEDC channel i, 50Hz, 14-bit.
// Order: FL_hip, FL_knee, FR_hip, FR_knee, RL_hip, RL_knee, RR_hip, RR_knee
// Mutable: servos_set_pin() can hot-swap a GPIO at runtime (tuning UI).
extern uint8_t SERVO_PINS[8];

// --- Servo Parameters ---
#define SERVO_FREQ_HZ       50
#define SERVO_MIN_US        500
#define SERVO_MAX_US        2500
#define SERVO_CENTER_US     1500

// --- Per-joint soft clamp limits (Hiwonder pwm_servo.cpp, asymmetric for mirrored mounts) ---
// Tighter than SERVO_MIN/MAX. Applied in IK layer to prevent driving joints into mechanical stops.
// Hardware safety net (SERVO_MIN/MAX) is enforced separately in servo_write_us().
// Order: FL_hip(0), FL_knee(1), FR_hip(2), FR_knee(3), RL_hip(4), RL_knee(5), RR_hip(6), RR_knee(7)
#ifndef SERVO_JOINT_CLAMP_DEFINED
#define SERVO_JOINT_CLAMP_DEFINED
static const uint16_t SERVO_JOINT_MIN_US[8] = {500, 900, 700, 1100, 500,  900, 700, 1100};
static const uint16_t SERVO_JOINT_MAX_US[8] = {2300, 1900, 2500, 2100, 2300, 1900, 2500, 2100};
#endif

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
    2096, 1621, 904, 1379, 2170, 1611, 830, 1389
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
    1800, 1500, 1202, 1500, 1870, 1500, 1165, 1500
};

// --- Battery ADC (VERIFIED: Hiwonder.__adcp = ADC(Pin(34), atten=3)) ---
#define BATTERY_ADC_PIN     34
#define BATTERY_DIVIDER     3.9f    // voltage divider ratio (~1:4 resistor divider)
#define BATTERY_LOW_MV      6700    // 1 Hz blink warning
#define BATTERY_CRITICAL_MV 6500    // 2 Hz blink warning
#define BATTERY_CUTOFF_MV   6400    // 2S LiPo cutoff (~3.2V/cell) — servos detach
#define BATTERY_HYSTERESIS_MV 100   // prevents rate chatter near thresholds
// USB-only: battery switch OFF leaves ADC rail at ~3.5V via regulator leakage.
// Anything below this threshold is physically impossible for a connected 2S pack.
#define BATTERY_ABSENT_MV            4500  // below any plausible 2S reading
#define BATTERY_ABSENT_HYSTERESIS_MV 500   // must exceed ABSENT+500 to re-enter present
#define BATTERY_ABSENT_SAMPLES       3     // consecutive 1 Hz reads to latch state

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
#define GAIT_SWING_TIME_MS      150     // ms per swing phase (foot in air) — Hiwonder default
#define GAIT_STAND_TIME_MS      200     // ms per stance phase (foot on ground) — Hiwonder default
#define GAIT_SPEED_ACCEL_PER_S  3.0f   // speed units/s — 0→1 ramp in ~330ms
#define GAIT_PARAM_RAMP_S       0.4f   // gait param change converges in ~400ms

// --- Return-to-stand taper ---
// When gait transitions back to STAND/STOP, servo positions are interpolated
// from wherever they are to the target standing pose over this duration.
// Prevents violent snap-to-standing on control release.
#define STAND_RETURN_MS         600     // ms to blend from gait position to standing

// --- Servo polarity overrides ---
// The IK auto-derives polarity from standing pulse: >1500 → +1, <1500 → -1.
// If a servo is physically mounted in reverse, override with +1 or -1 here.
// 0 = use auto-derived polarity.
// clang-format off
#ifndef IK_POLARITY_OVERRIDE_DEFINED
#define IK_POLARITY_OVERRIDE_DEFINED
static const int8_t SERVO_POLARITY_OVERRIDE[8] = {
    0,   // 0 FL_hip  — auto (+1, standing=2096)
    0,   // 1 FL_knee — auto (+1, standing=1621)
    0,   // 2 FR_hip  — auto (-1, standing=904)   VERIFIED: pulse↓ → forward swing ✓
    0,   // 3 FR_knee — auto (-1, standing=1379)
    0,   // 4 RL_hip  — auto (+1, standing=2170)  UNVERIFIED post-swap
    0,   // 5 RL_knee — auto (+1, standing=1611)  UNVERIFIED post-swap
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

// --- LED Brightness ---
// Scale is 0-255. Stock firmware uses full 0-255 range.
#define LED_BRIGHTNESS  40  // dim but clearly visible at ~16% max

// --- Auxiliary Servo Ports (expansion connectors SERVO7, SERVO9, SERVO11) ---
// Schematic ports on VIN rail: SERVO7=GPIO15(R37), SERVO9=GPIO0(R38), SERVO11=GPIO12(R39).
// GPIO 0 is a boot-mode strapping pin (LOW=download); safe after boot, must be HIGH at power-on.
// GPIO 12 is a flash-voltage strapping pin (HIGH=1.8V); treat carefully — series R39 (5.1K) is present.
// Disabled by default. Set AUX_SERVOS_ENABLED=1 to activate.
#ifndef AUX_SERVOS_ENABLED
#define AUX_SERVOS_ENABLED    0   // opt-in: set to 1 to enable expansion servo ports
#endif
#define AUX_SERVO_COUNT       3
// GPIO pins for aux servo firmware indices 8, 9, 10
static constexpr uint8_t AUX_SERVO_PINS[AUX_SERVO_COUNT]    = {15, 0, 12};

// --- Auxiliary GPIO ---
// J5 expansion header (IO32, IO33) and UART0 header (TX=IO1, RX=IO3).
// All four pins are unused by the base firmware; gpio_aux_* functions gate access.
#define AUX_GPIO_ENABLED 1

// --- LED Colors (0-255 scale) ---
// Lavender: R=180, G=110, B=255 at full brightness, scaled to LED_BRIGHTNESS
#define LED_R_LAVENDER  ((180 * LED_BRIGHTNESS) / 255)  // ~28
#define LED_G_LAVENDER  ((110 * LED_BRIGHTNESS) / 255)  // ~17
#define LED_B_LAVENDER  LED_BRIGHTNESS                   // 40
