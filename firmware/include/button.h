#pragma once
#include <stdint.h>

enum class ButtonEvent : uint8_t { NONE, PRESS, RELEASE, LONG_PRESS };

// Snapshot of a single button_update() call.
// held_ms is 0 for PRESS and NONE events; it is the actual press duration
// for RELEASE and LONG_PRESS events (snapshotted before the state flip so
// it is valid even though s_stable is already false when the caller reads it).
struct ButtonResult {
    ButtonEvent event;
    uint32_t    held_ms;
};

void button_init();
// Call from sensor_task every ~10ms. Returns edge event + held duration.
// held_ms is meaningful only for RELEASE and LONG_PRESS events.
ButtonResult button_update(uint32_t now_ms);
// Returns ms since last press start (0 if not currently pressed / never pressed).
uint32_t button_held_ms();
