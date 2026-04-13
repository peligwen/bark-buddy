// firmware/test/mock_preferences.h
#pragma once
#include <cstring>
#include <cstdint>

// Minimal Preferences stub for native tests (replaces ESP32 <Preferences.h>)
class Preferences {
    int16_t store[9] = {};  // 9 slots: off0..off8
    bool opened = false;
public:
    bool begin(const char*, bool = false) { opened = true; return true; }
    void end() { opened = false; }
    int16_t getShort(const char* key, int16_t def = 0) {
        // Keys are "off0".."off8"
        if (key[0]=='o' && key[1]=='f' && key[2]=='f') {
            int idx = key[3] - '0';
            if (idx >= 0 && idx < 9) return store[idx];
        }
        return def;
    }
    size_t putShort(const char* key, int16_t val) {
        if (key[0]=='o' && key[1]=='f' && key[2]=='f') {
            int idx = key[3] - '0';
            if (idx >= 0 && idx < 9) { store[idx] = val; return 2; }
        }
        return 0;
    }
    bool clear() { memset(store, 0, sizeof(store)); return true; }
};
