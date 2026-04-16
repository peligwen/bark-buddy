#include "servos.h"
#include "config.h"
#include <Arduino.h>

// Hardware PWM via ESP32 LEDC peripheral.
// Each servo gets its own LEDC channel: servo index i → LEDC channel i.
// Uses old-style API (ledcSetup/ledcAttachPin/ledcWrite(channel,...)) compatible
// with espressif32 Arduino core 2.x.
// Zero CPU usage during pulse generation — no FreeRTOS task, no busy-wait.
// 14-bit resolution at 50Hz gives ~1.22us per tick (adequate for all servos).

static uint16_t current_us[8] = {0};
static bool attached = false;

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

bool servos_attach_at(const uint16_t pose[8]) {
#if !PINS_VERIFIED
    Serial.println("{\"type\":\"error\",\"msg\":\"PINS_VERIFIED=0, servos disabled\"}");
    return false;
#else
    if (attached) return false;
    for (int i = 0; i < 8; i++) {
        ledcSetup(i, SERVO_FREQ_HZ, LEDC_RESOLUTION);
        ledcAttachPin(SERVO_PINS[i], i);
    }
    attached = true;
    for (int i = 0; i < 8; i++) {
        uint16_t pos = clamp_us(pose[i]);
        current_us[i] = pos;
        ledcWrite(SERVO_PINS[i], us_to_duty(pos));
    }
    return true;
#endif
}

void servos_ramp_to(const uint16_t target[8], uint16_t duration_ms, uint8_t steps) {
    if (!attached) return;
    if (steps == 0) steps = 1;
    uint16_t start_us[8];
    for (int i = 0; i < 8; i++) {
        start_us[i] = current_us[i] > 0 ? current_us[i] : target[i];
    }
    for (int step = 0; step <= steps; step++) {
        float t = (float)step / (float)steps;
        for (int i = 0; i < 8; i++) {
            int16_t s = (int16_t)start_us[i];
            int16_t e = (int16_t)target[i];
            uint16_t pos = (uint16_t)(s + (int16_t)((float)(e - s) * t));
            pos = clamp_us(pos);
            current_us[i] = pos;
            ledcWrite(SERVO_PINS[i], us_to_duty(pos));
        }
        delay(duration_ms / steps);
    }
}

bool servos_shutdown_to_rest() {
    if (!attached) return false;
    servos_ramp_to(REST_POSE, SHUTDOWN_RAMP_MS, SHUTDOWN_RAMP_STEPS);
    delay(REST_SETTLE_MS);
    servos_detach_all();
    return true;
}

bool servos_init() {
    if (!servos_attach_at(REST_POSE)) return false;
    delay(BOOT_SETTLE_MS);
    servos_ramp_to(STANDING_POSE, SOFTSTART_DURATION_MS, SOFTSTART_STEPS);
    return true;
}

void servo_write_us(uint8_t index, uint16_t pulse_us) {
    if (!attached || index >= 8) return;
    pulse_us = clamp_us(pulse_us);
    current_us[index] = pulse_us;
    ledcWrite(SERVO_PINS[index], us_to_duty(pulse_us));
}


uint16_t servo_read_us(uint8_t index) {
    if (index >= 8) return 0;
    return current_us[index];
}

void servos_detach_all() {
    if (!attached) return;
    attached = false;
    for (int i = 0; i < 8; i++) {
        ledcDetachPin(SERVO_PINS[i]);
        current_us[i] = 0;
    }
}

bool servos_active() {
    return attached;
}

bool servos_shutdown_to_lying_down() {
    if (!attached) return false;
    servos_ramp_to(LYING_DOWN_POSE, SHUTDOWN_RAMP_MS, SHUTDOWN_RAMP_STEPS);
    delay(SHUTDOWN_SETTLE_MS);
    servos_detach_all();
    return true;
}

