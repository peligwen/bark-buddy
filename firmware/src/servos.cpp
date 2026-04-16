#include "servos.h"
#include "config.h"
#include <Arduino.h>

// Hardware PWM via ESP32 LEDC peripheral (old-style ledcSetup/ledcAttachPin API).
// Servo index i → LEDC channel i. Writes use ledcWrite(pin, duty) — pin-based.
// 50Hz, 14-bit resolution (~1.22us/tick). Zero CPU cost during pulse generation.
// Engage: attach at REST_POSE -> ramp to STANDING_POSE (non-blocking, millis-based).
// Disengage: ramp to REST_POSE -> detach.

static uint16_t current_us[8] = {0};
static bool s_engaged = false;
static bool s_ramping = false;
static bool s_battery_cutoff = false;

// Ramp state
static uint32_t s_ramp_start_ms = 0;
static uint32_t s_ramp_duration_ms = 0;
static uint16_t s_ramp_from[8] = {};
static uint16_t s_ramp_to_pose[8] = {};
static bool s_ramp_is_engage = false;  // true=engage, false=disengage

static uint16_t clamp_us(uint16_t us) {
    if (us < SERVO_MIN_US) return SERVO_MIN_US;
    if (us > SERVO_MAX_US) return SERVO_MAX_US;
    return us;
}

// Convert pulse width in microseconds to LEDC duty count.
// period_us = 1,000,000 / SERVO_FREQ_HZ = 20,000us
static uint32_t us_to_duty(uint16_t pulse_us) {
    return (uint32_t)((uint64_t)pulse_us * LEDC_MAX_DUTY / (1000000UL / SERVO_FREQ_HZ));
}

bool servos_engage_start() {
#if !PINS_VERIFIED
    Serial.println("{\"type\":\"error\",\"msg\":\"PINS_VERIFIED=0\"}");
    return false;
#else
    if (s_engaged || s_ramping || s_battery_cutoff) return false;
    for (int i = 0; i < 8; i++) {
        ledcSetup(i, SERVO_FREQ_HZ, LEDC_RESOLUTION);
        ledcAttachPin(SERVO_PINS[i], i);
        uint16_t pos = clamp_us(REST_POSE[i]);
        current_us[i] = pos;
        ledcWrite(SERVO_PINS[i], us_to_duty(pos));
    }
    s_engaged = true;
    for (int i = 0; i < 8; i++) {
        s_ramp_from[i] = REST_POSE[i];
        s_ramp_to_pose[i] = STANDING_POSE[i];
    }
    s_ramp_start_ms = millis();
    s_ramp_duration_ms = SOFTSTART_DURATION_MS;
    s_ramp_is_engage = true;
    s_ramping = true;
    return true;
#endif
}

void servos_disengage_start() {
    if (!s_engaged && !s_ramping) return;
    if (s_ramping && !s_ramp_is_engage) return;  // already disengaging — do not restart ramp
    // If mid-engage-ramp, freeze current position as start of disengage ramp
    for (int i = 0; i < 8; i++) {
        s_ramp_from[i] = (current_us[i] > 0) ? current_us[i] : STANDING_POSE[i];
        s_ramp_to_pose[i] = REST_POSE[i];
    }
    s_ramp_start_ms = millis();
    s_ramp_duration_ms = SHUTDOWN_RAMP_MS;
    s_ramp_is_engage = false;
    s_ramping = true;
}

RampResult servos_ramp_tick(uint32_t now_ms) {
    if (!s_ramping) return RampResult::NONE;
    uint32_t elapsed = now_ms - s_ramp_start_ms;
    if (elapsed >= s_ramp_duration_ms) {
        // Write final target
        for (int i = 0; i < 8; i++) {
            servo_write_us(i, s_ramp_to_pose[i]);
        }
        bool was_engage = s_ramp_is_engage;
        s_ramping = false;
        if (!was_engage) {
            // Disengage complete — detach
            servos_detach_all();
        }
        return was_engage ? RampResult::ENGAGE_COMPLETE : RampResult::DISENGAGE_COMPLETE;
    }
    float t = (float)elapsed / (float)s_ramp_duration_ms;
    for (int i = 0; i < 8; i++) {
        int16_t s = (int16_t)s_ramp_from[i];
        int16_t e = (int16_t)s_ramp_to_pose[i];
        uint16_t pos = (uint16_t)(s + (int16_t)((float)(e - s) * t));
        servo_write_us(i, pos);
    }
    return RampResult::NONE;
}

bool servos_is_ramping() {
    return s_ramping;
}

bool servos_engaged() {
    return s_engaged;
}

void servo_write_us(uint8_t index, uint16_t pulse_us) {
    if (!s_engaged || index >= 8) return;
    pulse_us = clamp_us(pulse_us);
    current_us[index] = pulse_us;
    ledcWrite(SERVO_PINS[index], us_to_duty(pulse_us));
}

uint16_t servo_read_us(uint8_t index) {
    if (index >= 8) return 0;
    return current_us[index];
}

void servos_detach_all() {
    if (!s_engaged && !s_ramping) return;
    s_engaged = false;
    s_ramping = false;
    for (int i = 0; i < 8; i++) {
        ledcDetachPin(SERVO_PINS[i]);
        current_us[i] = 0;
    }
}

void servos_set_battery_cutoff() {
    s_battery_cutoff = true;
    servos_detach_all();
}

bool servos_battery_cutoff() {
    return s_battery_cutoff;
}
