// firmware/src/comms_out.cpp
// Outbound JSON queue — pre-serialize on sender, drain in main loop.
//
// Mirrors the LedCmd/I2cCmd queue pattern from sensor_task. Queue depth 8;
// items dropped silently if full (non-blocking xQueueSend with timeout 0).

#include "comms_out.h"
#include "comms.h"

#ifndef MOCK_FIRMWARE
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#else
#include "freertos_shim.h"
#endif

struct OutboundJson {
    char buf[192];
};

static QueueHandle_t s_out_queue = nullptr;

void comms_out_init() {
    s_out_queue = xQueueCreate(8, sizeof(OutboundJson));
}

void comms_emit_json_from_task(const JsonDocument& doc) {
    if (!s_out_queue) return;
    OutboundJson item;
    serializeJson(doc, item.buf, sizeof(item.buf));
    // ArduinoJson does not null-terminate when truncating; force it.
    item.buf[sizeof(item.buf) - 1] = '\0';
    xQueueSend(s_out_queue, &item, 0);
}

void comms_out_drain() {
    if (!s_out_queue) return;
    OutboundJson item;
    while (xQueueReceive(s_out_queue, &item, 0) == pdTRUE) {
        comms_write_line(item.buf);
    }
}
