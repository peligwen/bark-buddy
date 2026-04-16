#pragma once
// Arduino.h shim for mock firmware build.
// Pulls in mock_arduino.h from firmware/test/, which is on the include path.
// We also include the real-time millis/delay variant.
#define MOCK_REAL_TIME 1
#include "mock_arduino.h"

// Additional Arduino-isms needed by firmware source
#ifndef HIGH
#define HIGH 1
#define LOW  0
#endif
#ifndef INPUT
#define INPUT  0
#define OUTPUT 1
#define INPUT_PULLUP 2
#endif

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t) { return 0; }

inline void yield() {}
inline void esp_task_wdt_reset() {}

// Bring in platform shims
#include "WiFi.h"
#include "ESPmDNS.h"
#include "Wire.h"
#include "esp_compat.h"
#include "freertos_shim.h"
#include "Preferences.h"
