// firmware/test/mock_preferences.h
#pragma once
// Block the real Arduino Preferences.h from loading — we define our own stub.
#define _PREFERENCES_H_
#include <cstring>
#include <cstdint>

// Minimal Preferences stub for native tests (replaces ESP32 <Preferences.h>).
// Supports the operations used by offsets.cpp (int, short, float, remove, isKey).
class Preferences {
    static const int CAP = 32;
    struct Entry {
        char  key[16];
        enum class T : uint8_t { NONE, INT16, INT32, FLOAT } type = T::NONE;
        union { int16_t i16; int32_t i32; float f32; } val = {};
    };
    static Entry s_store[CAP];  // shared across namespace instances (simplified mock)
    bool opened = false;

    static int find(const char* key) {
        for (int i = 0; i < CAP; i++) {
            if (s_store[i].type != Entry::T::NONE && strcmp(s_store[i].key, key) == 0)
                return i;
        }
        return -1;
    }
    static int alloc(const char* key) {
        int idx = find(key);
        if (idx >= 0) return idx;
        for (int i = 0; i < CAP; i++) {
            if (s_store[i].type == Entry::T::NONE) {
                strncpy(s_store[i].key, key, sizeof(s_store[i].key) - 1);
                s_store[i].key[sizeof(s_store[i].key) - 1] = '\0';
                return i;
            }
        }
        return -1;
    }

public:
    bool begin(const char* /*ns*/, bool /*readOnly*/ = false) { opened = true; return true; }
    void end() { opened = false; }

    bool isKey(const char* key) { return find(key) >= 0; }

    int16_t getShort(const char* key, int16_t def = 0) {
        int i = find(key);
        return (i >= 0 && s_store[i].type == Entry::T::INT16) ? s_store[i].val.i16 : def;
    }
    size_t putShort(const char* key, int16_t val) {
        int i = alloc(key);
        if (i < 0) return 0;
        s_store[i].type = Entry::T::INT16;
        s_store[i].val.i16 = val;
        return 2;
    }

    int32_t getInt(const char* key, int32_t def = 0) {
        int i = find(key);
        if (i < 0) return def;
        if (s_store[i].type == Entry::T::INT32) return s_store[i].val.i32;
        if (s_store[i].type == Entry::T::INT16) return s_store[i].val.i16;
        return def;
    }
    size_t putInt(const char* key, int32_t val) {
        int i = alloc(key);
        if (i < 0) return 0;
        s_store[i].type = Entry::T::INT32;
        s_store[i].val.i32 = val;
        return 4;
    }

    float getFloat(const char* key, float def = 0.0f) {
        int i = find(key);
        return (i >= 0 && s_store[i].type == Entry::T::FLOAT) ? s_store[i].val.f32 : def;
    }
    size_t putFloat(const char* key, float val) {
        int i = alloc(key);
        if (i < 0) return 0;
        s_store[i].type = Entry::T::FLOAT;
        s_store[i].val.f32 = val;
        return 4;
    }

    bool remove(const char* key) {
        int i = find(key);
        if (i < 0) return false;
        s_store[i].type = Entry::T::NONE;
        s_store[i].key[0] = '\0';
        return true;
    }

    bool clear() {
        for (int i = 0; i < CAP; i++) s_store[i].type = Entry::T::NONE;
        return true;
    }
};

// Out-of-line definition for the static member
inline Preferences::Entry Preferences::s_store[Preferences::CAP] = {};
