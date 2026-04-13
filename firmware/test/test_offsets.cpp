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

    printf("{\"summary\":\"offsets_tests\",\"pass\":%d,\"fail\":%d}\n", pass, fail);
    return fail > 0 ? 1 : 0;
}
