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

// Simulated clock (simulated by default; real-time under MOCK_REAL_TIME)
#ifdef MOCK_REAL_TIME
#include <chrono>
#include <thread>
inline uint32_t millis() {
    static auto _start = std::chrono::steady_clock::now();
    return (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - _start).count();
}
inline void delay(unsigned long ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
#else
static unsigned long _mock_millis = 0;
inline unsigned long millis() { return _mock_millis; }
inline void delay(unsigned long ms) { _mock_millis += ms; }
#endif
#include <cstdlib>
inline uint32_t esp_random() { return (uint32_t)rand(); }
#ifndef MOCK_REAL_TIME
inline void mock_advance_ms(unsigned long ms) { _mock_millis += ms; }
inline void mock_reset_clock() { _mock_millis = 0; }
#endif

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

// LEDC stubs — 3.x pin-based API (ledcAttach/ledcWrite(pin,...)/ledcDetach).
// All calls use GPIO pin numbers directly; no channel tracking needed.
inline void ledcAttach(uint8_t pin, uint32_t freq, uint8_t resolution) { (void)pin; (void)freq; (void)resolution; }
inline void ledcDetach(uint8_t pin) { _servo_duty[pin] = 0; }

#ifdef MOCK_FIRMWARE
namespace physics { void on_servo_duty(uint8_t pin, uint32_t duty); }
#endif

inline void ledcWrite(uint8_t pin, uint32_t duty) {
    _servo_duty[pin] = duty;
    if (_servo_log_count < MAX_CAPTURES) {
        _servo_log[_servo_log_count++] = {pin, duty, (unsigned long)millis()};
    }
#ifdef MOCK_FIRMWARE
    physics::on_servo_duty(pin, duty);
#endif
}


// ADC stub — runtime-settable for testing battery cutoff and absent-state paths
inline int _mock_battery_raw = 3200;  // default: ~7.4V after divider
inline int analogRead(uint8_t pin) { (void)pin; return _mock_battery_raw; }
inline void mock_set_battery_raw(int raw) { _mock_battery_raw = raw; }

// Serial stub
struct MockSerial {
    void begin(unsigned long) {}
    void end() {}
    void println(const char* s = "") { printf("%s\n", s); }
    void print(const char* s) { printf("%s", s); }
    size_t write(uint8_t c) { printf("%c", c); return 1; }
    size_t write(const uint8_t* buf, size_t n) { fwrite(buf, 1, n, stdout); return n; }
    void flush() { fflush(stdout); }
    int available() { return 0; }
    char read() { return 0; }
    operator bool() { return true; }
};
static MockSerial Serial;

// Arduino String type — thin wrapper around std::string
#include <string>
struct String : std::string {
    using std::string::string;
    String(const std::string& s) : std::string(s) {}
    bool isEmpty() const { return empty(); }
    int toInt() const { return empty() ? 0 : std::stoi(*this); }
    const char* c_str() const { return std::string::c_str(); }
};

// constrain
template<typename T>
inline T constrain(T val, T lo, T hi) { return std::min(std::max(val, lo), hi); }

// Override PINS_VERIFIED for testing
#undef PINS_VERIFIED
#define PINS_VERIFIED 1

// Arduino-ESP32 version macros (real values come from the SDK; stub for host tests)
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 3
#define ESP_ARDUINO_VERSION_MINOR 3
#define ESP_ARDUINO_VERSION_PATCH 8
#endif
