# Hardware

Reference: `firmware/include/config.h` is the authoritative source for pin numbers and timing constants.
Manufacturer PDFs: `docs/manufacturer/`.

## Main Controller

**ESP32-WROOM-32D** (D0WD chip, dual-core Xtensa LX6, 240 MHz, 4 MB flash).
The separate ESP32-S3 on the vision module is for the camera only and is not connected.

## Servo Layout

Eight PWM servos controlled via ESP32 LEDC (14-bit, 50 Hz). Driven via arduino-esp32 3.x pin-based LEDC API (`ledcAttach`/`ledcWrite(pin)`); no channel bookkeeping required — the core assigns channels internally.
Default GPIO assignments from `firmware/src/servos.cpp:SERVO_PINS[8]`.

| Index | Name | Default GPIO | Standing (μs) | Rest (μs) | Polarity |
|-------|------|--------------|---------------|-----------|----------|
| 0 | FL_hip | 25 | 2096 | 1800 | auto (+1) |
| 1 | FL_knee | 26 | 1621 | 1500 | auto (+1) |
| 2 | FR_hip | 27 | 2170 | 1870 | auto (+1) |
| 3 | FR_knee | 14 | 1611 | 1500 | auto (+1) |
| 4 | RL_hip | 16 | 904 | 1202 | auto (−1) |
| 5 | RL_knee | 17 | 1379 | 1500 | auto (−1) |
| 6 | RR_hip | 15 | 830 | 1165 | override +1 |
| 7 | RR_knee | 2 | 1389 | 1500 | override −1 |

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
| Low | < 6700 mV | LED blinks 1 Hz |
| Critical | < 6500 mV | LED blinks 2 Hz |
| Cutoff | < 6400 mV | Servos detach; state latched until reboot |
| Hysteresis | 100 mV | Prevents rapid threshold crossing |

## WiFi

TCP port 9000. Credentials in `firmware/include/config_local.h` (gitignored).
Copy `config_local.h.example` → `config_local.h` and fill in SSID/password.
mDNS service name: `_mechdog._tcp`.
