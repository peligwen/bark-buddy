// firmware/src/command_handlers_probe.cpp
//
// cmd_probe_pin — wiggle a GPIO ±PROBE_AMPLITUDE_US around a center pulse to
// physically identify which servo is on which pin. Used during initial setup
// or after a wiring change.
//
// Synchronous: takes ~PROBE_CYCLES * PROBE_HALF_PERIOD_MS * 2 ≈ 2 s of
// blocking delay() calls. The heartbeat watchdog tolerates this; clients
// should treat it as such.
//
// Rejected reasons (ack ok=false, error):
//   invalid_pin            — sentinel / missing
//   pin_reserved:<reason>  — pin owned by a firmware subsystem
//   servo_pin_engaged      — pin is currently driving a live servo

#include "command_handlers_internal.h"
#include "comms.h"
#include "config.h"
#include "protocol.h"
#include "pin_registry.h"
#include "servos.h"

#include <Arduino.h>
#include <ArduinoJson.h>

static uint32_t probe_us_to_duty(uint16_t pulse_us) {
    // period = 20000us (50Hz); 14-bit = 16383 ticks max
    return (uint32_t)((uint64_t)pulse_us * 16383UL / 20000UL);
}

void handle_cmd_probe_pin(const JsonDocument& doc) {
    uint8_t  pin    = doc["pin"]      | 255;
    uint16_t center = doc["pulse_us"] | 1500;

    if (pin == 255) {
        send_ack(MSG_CMD_PROBE_PIN, false, "invalid_pin");
        return;
    }
    const char* reserved_reason = nullptr;
    if (pin_is_reserved(pin, &reserved_reason)) {
        char err_buf[32];
        snprintf(err_buf, sizeof(err_buf), "pin_reserved:%s",
                 reserved_reason ? reserved_reason : "unknown");
        send_ack(MSG_CMD_PROBE_PIN, false, err_buf);
        return;
    }
    if (servos_engaged()) {
        for (int i = 0; i < 8; i++) {
            if (SERVO_PINS[i] == pin) {
                send_ack(MSG_CMD_PROBE_PIN, false, "servo_pin_engaged");
                return;
            }
        }
#if AUX_SERVOS_ENABLED
        for (int i = 0; i < AUX_SERVO_COUNT; i++) {
            if (AUX_SERVO_PINS[i] == pin) {
                send_ack(MSG_CMD_PROBE_PIN, false, "servo_pin_engaged");
                return;
            }
        }
#endif
    }
    if (center < 500)  center = 500;
    if (center > 2500) center = 2500;

    ledcAttach(pin, PROBE_FREQ_HZ, PROBE_RESOLUTION);

    for (int i = 0; i < PROBE_CYCLES; i++) {
        uint16_t hi = center + PROBE_AMPLITUDE_US;
        uint16_t lo = (center > PROBE_AMPLITUDE_US) ? (center - PROBE_AMPLITUDE_US) : 500u;
        ledcWrite(pin, probe_us_to_duty(hi));
        delay(PROBE_HALF_PERIOD_MS);
        ledcWrite(pin, probe_us_to_duty(lo));
        delay(PROBE_HALF_PERIOD_MS);
    }
    ledcWrite(pin, probe_us_to_duty(center));
    delay(50);
    ledcDetach(pin);

    JsonDocument resp;
    resp["type"]     = MSG_ACK;
    resp["ref_type"] = MSG_CMD_PROBE_PIN;
    resp["ok"]       = true;
    resp["pin"]      = pin;
    send_json(resp);
}
