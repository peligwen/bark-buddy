# Hardware

Reference: `firmware/include/config.h` is the authoritative source for pin numbers and timing constants.
Manufacturer PDFs: `docs/manufacturer/`.

## Main Controller

**ESP32-WROOM-32D** (D0WD chip, dual-core Xtensa LX6, 240 MHz, 4 MB flash).
The vision module is a second, independent ESP32-S3 board; see [Vision Module](#vision-module).

## Servo Layout

Eight PWM servos controlled via ESP32 LEDC (14-bit, 50 Hz). Driven via arduino-esp32 3.x pin-based LEDC API (`ledcAttach`/`ledcWrite(pin)`); no channel bookkeeping required — the core assigns channels internally.
Default GPIO assignments from `firmware/src/servos.cpp:SERVO_PINS[8]`.

| Index | Name | Default GPIO | Standing (μs) | Rest (μs) | Polarity |
|-------|------|--------------|---------------|-----------|----------|
| 0 | FL_hip | 25 | 2096 | 1800 | auto (+1) |
| 1 | FL_knee | 26 | 1621 | 1500 | auto (+1) |
| 2 | FR_hip | 16 | 904 | 1202 | auto (−1) VERIFIED |
| 3 | FR_knee | 17 | 1379 | 1500 | auto (−1) |
| 4 | RL_hip | 27 | 2170 | 1870 | auto (+1) UNVERIFIED |
| 5 | RL_knee | 14 | 1611 | 1500 | auto (+1) UNVERIFIED |
| 6 | RR_hip | 15 | 830 | 1165 | override +1 |
| 7 | RR_knee | 2 | 1389 | 1500 | override −1 |

Nudge test (2026-04-20) found indices 2–3 and 4–5 had the wrong GPIOs for their labels. Fix applied: swapped `SERVO_PINS`, `STANDING_POSE`, and `REST_POSE` at slots [2,3] ↔ [4,5]. RL polarity at indices 4–5 is auto-derived but not yet verified on hardware — update once confirmed.

**Polarity:** derived from standing pulse relative to 1500 μs (>1500 → +1, <1500 → −1).
RR_hip/RR_knee are physically mounted inverted; polarity overrides correct the IK.

**Lying-down pose:** all servos at 1500 μs (safe neutral on boot and shutdown).

**Pulse range:** 500–2500 μs. Center: 1500 μs.
Softstart: 2000 ms ramp from lying-down to standing on engage.
Disengage ramp: 1500 ms ramp to rest pose, then PWM cut.

GPIO assignments are mutable at runtime via `cmd_servo_pin`. Changes are not persisted.

## I2C Buses

| Bus | Instance | SDA | SCL | Devices |
|-----|----------|-----|-----|---------|
| 1 | Wire | GPIO 22 | GPIO 23 | QMI8658 IMU (0x6A), ultrasonic sonar (0x77) |
| 2 | Wire1 | GPIO 19 | GPIO 13 | J4 expansion connector |

Both buses at 400 kHz. I2C address 0x6A confirmed for QMI8658 (not 0x6B).

## IMU Axis Convention

QMI8658 at I2C address 0x6A on bus 1. Output after complementary filter:

| Axis | Positive direction |
|------|-------------------|
| Pitch | Nose up |
| Roll | Left side up |
| Yaw | Integrated gyro — drifts; not used for localization |

Accelerometer: m/s² body frame. Gyroscope: deg/s body frame. IMU data-ready interrupt: GPIO 35 (input-only).

## Other Peripherals

| Peripheral | GPIO | Notes |
|------------|------|-------|
| Battery ADC | 34 | ~4:1 resistor divider; cutoff at 6400 mV (3.2 V/cell, 2S LiPo) |
| Button K1 | 5 | Active-LOW, 10 kΩ pullup; debounce 20 ms; long-press ≥ 1000 ms |
| IMU interrupt | 35 | QMI8658 INT2 data-ready; input-only pin |
| Onboard LED | 18 | Active-LOW; `cmd_led {led:0}` controls it; brightness capped at 40/255 |
| Buzzer | 21 | NPN transistor drive; LEDC pin-based (arduino-esp32 3.x) |

## Auxiliary GPIO

J5 expansion header: GPIO 32, 33. UART0 header: GPIO 1 (TX), 3 (RX).
All four are accessible via `cmd_gpio`. Enabled by `AUX_GPIO_ENABLED=1` (default on).

## Auxiliary Servo Ports

Expansion connectors SERVO7/9/11 mapped to GPIO 15, 0, 12.
Disabled by default (`AUX_SERVOS_ENABLED=0`).
GPIO 0 is a boot-mode strapping pin (must be HIGH at power-on).
GPIO 12 is a flash-voltage strapping pin (series 5.1 kΩ resistor present).

## LEDC PWM Configuration

14-bit resolution: 16384 ticks per 20 ms period (50 Hz).

arduino-esp32 3.x pin-based API: servos, buzzer, and probe are attached with `ledcAttach(pin, freq, resolution)` and driven with `ledcWrite(pin, duty)`. Channel assignment is handled internally by the core; no channel constants are required in firmware code.

## Battery Thresholds

| Level | Voltage | Behavior |
|-------|---------|----------|
| Absent | < 4500 mV | Battery switch off / USB-only. LED off; cutoff latch suppressed (or cleared if previously set). `telem_battery.present = false`. Detected after 3 consecutive 1 Hz samples. |
| Low | < 6700 mV | LED blinks 1 Hz |
| Critical | < 6500 mV | LED blinks 2 Hz |
| Cutoff | < 6400 mV | Servos detach; state latched until reboot (or until battery switch is turned off) |
| Hysteresis | 100 mV | Prevents rapid threshold crossing (voltage-based states) |
| Absent hysteresis | 500 mV | Must exceed 5000 mV to re-enter Present from Absent |

**Why 4500 mV?** A connected 2S LiPo reads at minimum ~6000 mV (2 × 3.0 V/cell absolute floor). With the battery switch OFF and USB powering the ESP32, regulator leakage holds the ADC rail at ~3500 mV. The 4500 mV threshold sits safely between these ranges.

## Vision Module

Hiwonder ESP32-S3 AI Vision Module. It runs the firmware in `camera/` and the host reaches
it only over WiFi. Flash over its own USB-C port with `bark flash camera`.

Power is 5 V, through USB-C or the module's 4-pin port (Hiwonder specifies a 5 V 2 A
supply). The 4-pin port also carries I2C; `camera/` leaves those pins unconfigured, so the
module does not appear on the main controller's bus.

| Item | Value | Source |
|------|-------|--------|
| MCU | ESP32-S3, 8 MB flash (QIO 80 MHz), OPI PSRAM | Hiwonder Arduino IDE setup screenshot |
| Camera bus | XCLK 15, SIOD 4, SIOC 5, D0–D7 = 11, 9, 8, 10, 12, 18, 17, 16, VSYNC 6, HREF 7, PCLK 13; no PWDN/RESET | `CAMERA_MODEL_ESP32S3_EYE` in Hiwonder's `FaceDetection` source |
| XCLK | 15 MHz | same |
| Sensor | 2 MP; part not documented — firmware reads the PID at boot and reports it at `GET /` | retailer listing |
| Stock firmware | 320×240 stream; I2C slave at 0x52 for face/colour results | Hiwonder wiki |

UNVERIFIED on this unit: everything in the table comes from Hiwonder's published material
for the module, and `camera/` compiles against it, but it has not yet been run on hardware.
Record here once known: the sensor PID, how the module is powered on this dog, and the
lens height above the floor with the dog standing.

The stock firmware and Hiwonder's flash tool are linked from the appendix of
<https://wiki.hiwonder.com/projects/ESP32-S3/en/latest/> if the module needs restoring.

## WiFi

TCP port 9000. Credentials in `firmware/include/config_local.h` (gitignored).
Copy `config_local.h.example` → `config_local.h` and fill in SSID/password.
mDNS service name: `_mechdog._tcp`.

The vision module joins the same network with the same `config_local.h` credentials and
answers on HTTP port 80 as `bark-cam.local`.
