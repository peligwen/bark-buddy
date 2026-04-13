#include "offsets.h"

#if defined(ESP_PLATFORM)
#include <Preferences.h>
#else
#include "../test/mock_preferences.h"
#include <algorithm>
// local constrain for native
static inline int constrain_i(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
#endif

static const int NUM_SERVOS = 8;
static const char* NVS_NAMESPACE = "servo_cal";

static int16_t offsets[NUM_SERVOS] = {};
static Preferences prefs;

void offsets_init() {
    prefs.begin(NVS_NAMESPACE, /*readOnly=*/false);
    for (int i = 0; i < NUM_SERVOS; i++) {
        char key[5];
        key[0] = 'o'; key[1] = 'f'; key[2] = 'f';
        key[3] = (char)('0' + i); key[4] = '\0';
        offsets[i] = prefs.getShort(key, 0);
    }
    prefs.end();
}

int16_t offset_get(uint8_t servo_idx) {
    if (servo_idx >= NUM_SERVOS) return 0;
    return offsets[servo_idx];
}

void offset_set(uint8_t servo_idx, int16_t offset_us) {
    if (servo_idx >= NUM_SERVOS) return;
    // Clamp to ±500us — apply_offset already clamps final pulse to [500,2500]
    // but we prevent extreme stored values causing confusing behaviour
    if (offset_us >  500) offset_us =  500;
    if (offset_us < -500) offset_us = -500;
    offsets[servo_idx] = offset_us;
}

void offsets_save() {
    prefs.begin(NVS_NAMESPACE, /*readOnly=*/false);
    for (int i = 0; i < NUM_SERVOS; i++) {
        char key[5];
        key[0] = 'o'; key[1] = 'f'; key[2] = 'f';
        key[3] = (char)('0' + i); key[4] = '\0';
        prefs.putShort(key, offsets[i]);
    }
    prefs.end();
}

// NOTE: prefs.clear() wipes the entire "servo_cal" NVS namespace.
// Do not add other data to this namespace — it will be destroyed on reset.
void offsets_reset() {
    for (int i = 0; i < NUM_SERVOS; i++) {
        offsets[i] = 0;
    }
    prefs.begin(NVS_NAMESPACE, /*readOnly=*/false);
    prefs.clear();
    prefs.end();
}

uint16_t apply_offset(uint8_t servo_idx, uint16_t raw_us) {
    int val = (int)raw_us + (int)offset_get(servo_idx);
#if defined(ESP_PLATFORM)
    return (uint16_t)constrain(val, 500, 2500);
#else
    return (uint16_t)constrain_i(val, 500, 2500);
#endif
}
