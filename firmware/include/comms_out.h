// firmware/include/comms_out.h
// Outbound JSON queue — route sensor-task send_json through FreeRTOS queue.
//
// Callers on the sensor task use comms_emit_json_from_task(); the main loop
// drains the queue with comms_out_drain(), which writes via comms_write_line().
#pragma once
#include <ArduinoJson.h>

// Call once in setup(), before sensor_task_start().
void comms_out_init();

// Pre-serialize doc into an OutboundJson item and enqueue it (non-blocking).
// Safe to call from any FreeRTOS task. Drops silently if queue is full.
void comms_emit_json_from_task(const JsonDocument& doc);

// Drain all pending items from the queue and write them via comms_write_line().
// Call from main loop only.
void comms_out_drain();
