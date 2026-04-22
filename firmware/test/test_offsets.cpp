#include "mock_arduino.h"
#include "../src/offsets.cpp"  // include impl directly for native test
#include <cstdio>
#include <cmath>

int main() {
    int pass = 0, fail = 0;

    // Test 1: init zeros all offsets
    offsets_init();
    bool all_zero = true;
    for (int i = 0; i < 8; i++) if (offset_get(i) != 0) all_zero = false;
    printf("{\"test\":\"init_zeros\",\"pass\":%s}\n", all_zero ? "true" : "false");
    all_zero ? pass++ : fail++;

    // Test 2: set and get round-trip
    offset_set(3, 50);
    bool got = offset_get(3) == 50;
    printf("{\"test\":\"set_get\",\"pass\":%s}\n", got ? "true" : "false");
    got ? pass++ : fail++;

    // Test 3: apply_offset clamps to [500, 2500]
    offset_set(0, 500);
    uint16_t high = apply_offset(0, 2200);  // 2200+500=2700, clamp to 2500
    bool clamp_high = high == 2500;
    offset_set(0, -500);
    uint16_t low = apply_offset(0, 800);    // 800-500=300, clamp to 500
    bool clamp_low = low == 500;
    printf("{\"test\":\"apply_clamp\",\"high\":%d,\"low\":%d,\"pass\":%s}\n",
           high, low, (clamp_high && clamp_low) ? "true" : "false");
    (clamp_high && clamp_low) ? pass++ : fail++;

    // Test 4: reset zeroes all
    offset_set(5, -100);
    offsets_reset();
    bool zeroed = offset_get(5) == 0;
    printf("{\"test\":\"reset\",\"pass\":%s}\n", zeroed ? "true" : "false");
    zeroed ? pass++ : fail++;

    // Test 5: save/reload round-trip (via mock Preferences)
    offsets_init();
    offset_set(2, 75);
    offsets_save();
    offsets_init();  // reload
    bool reloaded = offset_get(2) == 75;
    printf("{\"test\":\"save_reload\",\"pass\":%s}\n", reloaded ? "true" : "false");
    reloaded ? pass++ : fail++;

    // Test 6: namespace isolation — write in "ns_a", must not be visible in "ns_b"
    {
        mock_prefs_reset();
        Preferences pa, pb;
        pa.begin("ns_a");
        pa.putInt("x", 42);
        pa.end();

        pb.begin("ns_b");
        bool isolated = !pb.isKey("x");
        pb.end();

        printf("{\"test\":\"ns_isolation\",\"pass\":%s}\n", isolated ? "true" : "false");
        isolated ? pass++ : fail++;
    }

    // Test 7: NVS migration — old "servo_cal"/"off0..off7" keys migrate to "mechdog"
    {
        mock_prefs_reset();

        // Pre-populate old namespace (as legacy code would have written)
        Preferences old;
        old.begin("servo_cal");
        for (int i = 0; i < 8; i++) {
            char key[8];
            snprintf(key, sizeof(key), "off%d", i);
            old.putShort(key, (int16_t)(10 * (i + 1)));
        }
        old.end();

        offsets_init();  // should migrate

        // Values should be readable via offset_get
        bool migrated = true;
        for (int i = 0; i < 8; i++) {
            if (offset_get(i) != (int16_t)(10 * (i + 1))) { migrated = false; break; }
        }
        // Old keys should be gone
        Preferences check_old;
        check_old.begin("servo_cal");
        bool old_cleared = !check_old.isKey("off0");
        check_old.end();

        printf("{\"test\":\"nvs_migration\",\"migrated\":%s,\"old_cleared\":%s,\"pass\":%s}\n",
               migrated ? "true" : "false", old_cleared ? "true" : "false",
               (migrated && old_cleared) ? "true" : "false");
        (migrated && old_cleared) ? pass++ : fail++;
    }

    // Test 8: read_offset probes int16 (C3 — stock firmware may use SHORT type)
    {
        mock_prefs_reset();

        // Write a SHORT type value directly into the new namespace
        Preferences p;
        p.begin("mechdog");
        p.putShort("offset_s1", 33);
        p.end();

        offsets_init();
        bool read_short = (offset_get(0) == 33);
        printf("{\"test\":\"read_offset_int16\",\"pass\":%s}\n", read_short ? "true" : "false");
        read_short ? pass++ : fail++;
    }

    printf("{\"summary\":\"offsets_tests\",\"pass\":%d,\"fail\":%d}\n", pass, fail);
    return fail > 0 ? 1 : 0;
}
