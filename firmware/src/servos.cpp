#include "servos.h"
#include "config.h"
#include <Arduino.h>

static uint16_t current_us[8] = {0};
static bool attached = false;

static uint16_t clamp_us(uint16_t us) {
    if (us < SERVO_MIN_US) return SERVO_MIN_US;
    if (us > SERVO_MAX_US) return SERVO_MAX_US;
    return us;
}

static uint32_t us_to_duty(uint16_t us) {
    // Convert microseconds to LEDC duty at given resolution
    // Period = 1000000 / SERVO_FREQ_HZ = 20000 us
    // Max duty = (1 << SERVO_RESOLUTION) - 1
    uint32_t max_duty = (1 << SERVO_RESOLUTION) - 1;
    return (uint32_t)((float)us / 20000.0f * max_duty);
}

bool servos_init() {
#if !PINS_VERIFIED
    Serial.println("{\"type\":\"error\",\"msg\":\"PINS_VERIFIED=0, servos disabled\"}");
    return false;
#else
    // Configure LEDC channels for each servo
    for (int i = 0; i < 8; i++) {
        ledcAttach(SERVO_PINS[i], SERVO_FREQ_HZ, SERVO_RESOLUTION);
        current_us[i] = SERVO_CENTER_US;
        ledcWrite(SERVO_PINS[i], us_to_duty(SERVO_CENTER_US));
    }
    attached = true;

    // Soft-start: ramp from center to standing pose
    for (int step = 0; step <= SOFTSTART_STEPS; step++) {
        float t = (float)step / (float)SOFTSTART_STEPS;
        for (int i = 0; i < 8; i++) {
            uint16_t target = STANDING_POSE[i];
            uint16_t pos = SERVO_CENTER_US + (uint16_t)((float)(target - SERVO_CENTER_US) * t);
            pos = clamp_us(pos);
            current_us[i] = pos;
            ledcWrite(SERVO_PINS[i], us_to_duty(pos));
        }
        delay(SOFTSTART_DURATION_MS / SOFTSTART_STEPS);
    }

    return true;
#endif
}

void servo_write_us(uint8_t index, uint16_t pulse_us) {
    if (!attached || index >= 8) return;
    pulse_us = clamp_us(pulse_us);
    current_us[index] = pulse_us;
    ledcWrite(SERVO_PINS[index], us_to_duty(pulse_us));
}

void servo_write_angle(uint8_t index, float angle_deg) {
    if (angle_deg < 0) angle_deg = 0;
    if (angle_deg > 180) angle_deg = 180;
    uint16_t us = SERVO_MIN_US + (uint16_t)(angle_deg / 180.0f * (SERVO_MAX_US - SERVO_MIN_US));
    servo_write_us(index, us);
}

uint16_t servo_read_us(uint8_t index) {
    if (index >= 8) return 0;
    return current_us[index];
}

void servos_detach_all() {
    if (!attached) return;
    for (int i = 0; i < 8; i++) {
        ledcDetach(SERVO_PINS[i]);
        current_us[i] = 0;
    }
    attached = false;
}

bool servos_active() {
    return attached;
}
