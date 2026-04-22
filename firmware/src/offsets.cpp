#include "offsets.h"
#include "config.h"

#if defined(ESP_PLATFORM) && !defined(UNIT_TEST)
#include <Preferences.h>
#else
#include "../test/mock_preferences.h"
#include <algorithm>
static inline int constrain_i(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
#endif

static const int  NUM_SERVOS    = 8;
static const char NVS_NS[]      = "mechdog";   // matches Hiwonder stock NVS namespace
static const char NVS_NS_OLD[]  = "servo_cal"; // legacy namespace — migrated at boot

static int16_t offsets[NUM_SERVOS] = {};
static Preferences prefs;

// Build the new-style key for servo index i: "offset_s1".."offset_s8"
static void servo_key(int i, char* buf, size_t len) {
    snprintf(buf, len, "offset_s%d", i + 1);
}

// Build the old-style key for servo index i: "off0".."off7"
static void servo_key_old(int i, char* buf, size_t len) {
    snprintf(buf, len, "off%d", i);
}

void offsets_init() {
    // Try new namespace first
    prefs.begin(NVS_NS, /*readOnly=*/false);
    char key[16];
    bool new_ns_has_data = false;
    for (int i = 0; i < NUM_SERVOS; i++) {
        servo_key(i, key, sizeof(key));
        if (prefs.isKey(key)) { new_ns_has_data = true; break; }
    }
    if (new_ns_has_data) {
        for (int i = 0; i < NUM_SERVOS; i++) {
            servo_key(i, key, sizeof(key));
            offsets[i] = (int16_t)prefs.getInt(key, 0);
        }
        prefs.end();
        return;
    }
    prefs.end();

    // Fall back to old namespace — migrate if found
    Preferences old_prefs;
    old_prefs.begin(NVS_NS_OLD, /*readOnly=*/false);
    bool old_ns_has_data = false;
    for (int i = 0; i < NUM_SERVOS; i++) {
        servo_key_old(i, key, sizeof(key));
        if (old_prefs.isKey(key)) { old_ns_has_data = true; break; }
    }
    if (old_ns_has_data) {
        int16_t migrated[NUM_SERVOS];
        for (int i = 0; i < NUM_SERVOS; i++) {
            servo_key_old(i, key, sizeof(key));
            migrated[i] = old_prefs.getShort(key, 0);
        }
        old_prefs.clear();  // wipe old namespace
        old_prefs.end();

        // Write into new namespace
        prefs.begin(NVS_NS, /*readOnly=*/false);
        for (int i = 0; i < NUM_SERVOS; i++) {
            servo_key(i, key, sizeof(key));
            prefs.putInt(key, migrated[i]);
            offsets[i] = migrated[i];
        }
        prefs.end();
        return;
    }
    old_prefs.end();

    // No stored data — start with zeros
    for (int i = 0; i < NUM_SERVOS; i++) offsets[i] = 0;
}

int16_t offset_get(uint8_t servo_idx) {
    if (servo_idx >= NUM_SERVOS) return 0;
    return offsets[servo_idx];
}

void offset_set(uint8_t servo_idx, int16_t offset_us) {
    if (servo_idx >= NUM_SERVOS) return;
    if (offset_us >  500) offset_us =  500;
    if (offset_us < -500) offset_us = -500;
    offsets[servo_idx] = offset_us;
}

void offsets_save() {
    char key[16];
    prefs.begin(NVS_NS, /*readOnly=*/false);
    for (int i = 0; i < NUM_SERVOS; i++) {
        servo_key(i, key, sizeof(key));
        prefs.putInt(key, offsets[i]);
    }
    prefs.end();
}

void offsets_reset() {
    for (int i = 0; i < NUM_SERVOS; i++) offsets[i] = 0;
    prefs.begin(NVS_NS, /*readOnly=*/false);
    // Clear only offset keys, not yaw trim
    char key[16];
    for (int i = 0; i < NUM_SERVOS; i++) {
        servo_key(i, key, sizeof(key));
        prefs.remove(key);
    }
    prefs.end();
}

uint16_t apply_offset(uint8_t servo_idx, uint16_t raw_us) {
    int val = (int)raw_us + (int)offset_get(servo_idx);
#if defined(ESP_PLATFORM) && !defined(UNIT_TEST)
    return (uint16_t)constrain(val, SERVO_MIN_US, SERVO_MAX_US);
#else
    return (uint16_t)constrain_i(val, SERVO_MIN_US, SERVO_MAX_US);
#endif
}

float yaw_trim_load() {
    prefs.begin(NVS_NS, /*readOnly=*/true);
    float val = prefs.getFloat("move_angle", 0.0f);
    prefs.end();
    if (val >  0.7f) val =  0.7f;
    if (val < -0.7f) val = -0.7f;
    return val;
}

void yaw_trim_save(float mul) {
    if (mul >  0.7f) mul =  0.7f;
    if (mul < -0.7f) mul = -0.7f;
    prefs.begin(NVS_NS, /*readOnly=*/false);
    prefs.putFloat("move_angle", mul);
    prefs.end();
}
