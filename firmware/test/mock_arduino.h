#pragma once
// Minimal Arduino stubs for host-side testing.
// Captures servo PWM output instead of driving real hardware.

// Satisfy any #include <Arduino.h> in firmware source files compiled into tests.
#define Arduino_h

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <algorithm>

// Simulated clock
static unsigned long _mock_millis = 0;
inline unsigned long millis() { return _mock_millis; }
inline void delay(unsigned long ms) { _mock_millis += ms; }
inline void mock_advance_ms(unsigned long ms) { _mock_millis += ms; }
inline void mock_reset_clock() { _mock_millis = 0; }

// Servo capture: records all PWM writes
struct ServoCapture {
    uint8_t pin;
    uint32_t duty;
    unsigned long timestamp;
};

static constexpr int MAX_CAPTURES = 100000;
static ServoCapture _servo_log[MAX_CAPTURES];
static int _servo_log_count = 0;
static uint32_t _servo_duty[64] = {};  // current duty per pin

inline void servo_log_reset() { _servo_log_count = 0; memset(_servo_duty, 0, sizeof(_servo_duty)); }

inline const ServoCapture* servo_log_entries() { return _servo_log; }
inline int servo_log_count() { return _servo_log_count; }

// LEDC stubs — supports both new-style (ledcAttach/ledcWrite(pin,...)) and
// old-style (ledcSetup/ledcAttachPin/ledcWrite(channel,...)) ESP32 Arduino APIs.
static uint8_t _channel_to_pin[16] = {};  // 0 means unmapped

// New-style API
inline void ledcAttach(uint8_t pin, uint32_t freq, uint8_t resolution) { (void)pin; (void)freq; (void)resolution; }
inline void ledcDetach(uint8_t pin) { _servo_duty[pin] = 0; }

// Old-style API
inline void ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution) { (void)channel; (void)freq; (void)resolution; }
inline void ledcAttachPin(uint8_t pin, uint8_t channel) { _channel_to_pin[channel] = pin; }
inline void ledcDetachPin(uint8_t pin) { _servo_duty[pin] = 0; }

// ledcWrite: first arg is channel (old-style) if mapped, else pin (new-style)
inline void ledcWrite(uint8_t pin_or_channel, uint32_t duty) {
    uint8_t pin = (pin_or_channel < 16 && _channel_to_pin[pin_or_channel] != 0)
                  ? _channel_to_pin[pin_or_channel]
                  : pin_or_channel;
    _servo_duty[pin] = duty;
    if (_servo_log_count < MAX_CAPTURES) {
        _servo_log[_servo_log_count++] = {pin, duty, _mock_millis};
    }
}

// ADC stub
inline int analogRead(uint8_t pin) { (void)pin; return 3200; } // ~7.4V with divider

// Serial stub
struct MockSerial {
    void begin(unsigned long) {}
    void println(const char* s = "") { printf("%s\n", s); }
    void print(const char* s) { printf("%s", s); }
    int available() { return 0; }
    char read() { return 0; }
    operator bool() { return true; }
};
static MockSerial Serial;

// constrain
template<typename T>
inline T constrain(T val, T lo, T hi) { return std::min(std::max(val, lo), hi); }

// Override PINS_VERIFIED for testing
#undef PINS_VERIFIED
#define PINS_VERIFIED 1
