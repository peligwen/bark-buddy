// firmware/include/comms.h
#pragma once
#include <ArduinoJson.h>

// Defined in main.cpp. Writes NDJSON to serial and (if connected) TCP client.
void send_json(const JsonDocument& doc);

// Convenience wrapper: sends {"type":"ack","ref_type":ref_type,"ok":ok[,"error":error]}
void send_ack(const char* ref_type, bool ok, const char* error = nullptr);

// Returns the remote IP of the currently connected TCP client (empty string if not connected).
String get_tcp_client_ip();

// Write a pre-serialized NDJSON line to serial and (if connected) TCP client.
// Used by comms_out_drain() to emit queued outbound messages.
void comms_write_line(const char* line);
