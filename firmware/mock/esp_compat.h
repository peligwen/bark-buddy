#pragma once
#include <cstdlib>
#include <cstdint>
struct ESPClass {
    uint32_t getChipId() { return 0xDEADBEEFu; }
    void restart() { exit(0); }
    uint32_t getFreeHeap() { return 200000; }
};
extern ESPClass ESP;
// esp_random() is already defined in mock_arduino.h (included earlier via Arduino.h).
// Do not redefine here to avoid duplicate-symbol errors.
#ifndef MOCK_ESP_RANDOM_DEFINED
#define MOCK_ESP_RANDOM_DEFINED
inline uint32_t esp_random() { return (uint32_t)rand(); }
#endif
