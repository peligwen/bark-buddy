#include <Arduino.h>
#include <ArduinoJson.h>
#include "protocol.h"

// --- Forward declarations ---
void handle_message(const JsonDocument& doc);
void send_ack(const char* ref_type, bool ok, const char* error = nullptr);
void send_pong();
void send_telem_imu();
void send_telem_status();
void send_json(const JsonDocument& doc);
void check_heartbeat();

// --- State ---
static char rx_buffer[MAX_MESSAGE_SIZE];
static size_t rx_pos = 0;
static unsigned long last_msg_received = 0;
static unsigned long last_imu_send = 0;
static unsigned long last_status_send = 0;
static bool connected = false;
static bool balance_enabled = true;

// --- Direction helpers ---
Direction direction_from_string(const char* str) {
    if (strcmp(str, "forward") == 0)  return Direction::FORWARD;
    if (strcmp(str, "backward") == 0) return Direction::BACKWARD;
    if (strcmp(str, "left") == 0)     return Direction::LEFT;
    if (strcmp(str, "right") == 0)    return Direction::RIGHT;
    return Direction::STOP;
}

const char* direction_to_string(Direction dir) {
    switch (dir) {
        case Direction::FORWARD:  return "forward";
        case Direction::BACKWARD: return "backward";
        case Direction::LEFT:     return "left";
        case Direction::RIGHT:    return "right";
        case Direction::STOP:     return "stop";
    }
    return "stop";
}

// --- Setup ---
void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) { delay(10); }

    last_msg_received = millis();
    connected = false;

    // TODO: Initialize servos (servos.cpp)
    // TODO: Initialize IMU (balance.cpp)
    // TODO: Initialize gait engine (gait.cpp)
}

// --- Main loop ---
void loop() {
    unsigned long now = millis();

    // Read incoming serial data (NDJSON: one message per line)
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            rx_buffer[rx_pos] = '\0';
            if (rx_pos > 0) {
                JsonDocument doc;
                DeserializationError err = deserializeJson(doc, rx_buffer);
                if (!err) {
                    last_msg_received = now;
                    connected = true;
                    handle_message(doc);
                }
            }
            rx_pos = 0;
        } else if (rx_pos < MAX_MESSAGE_SIZE - 1) {
            rx_buffer[rx_pos++] = c;
        }
    }

    // Check heartbeat timeout
    check_heartbeat();

    // Stream telemetry when connected
    if (connected) {
        if (now - last_imu_send >= TELEM_IMU_INTERVAL_MS) {
            send_telem_imu();
            last_imu_send = now;
        }
        if (now - last_status_send >= TELEM_STATUS_INTERVAL_MS) {
            send_telem_status();
            last_status_send = now;
        }
    }

    // TODO: Run balance PID loop (balance.cpp)
    // TODO: Run gait engine step (gait.cpp)
}

// --- Message handler ---
void handle_message(const JsonDocument& doc) {
    const char* type = doc["type"];
    if (!type) return;

    if (strcmp(type, MSG_PING) == 0) {
        send_pong();
    }
    else if (strcmp(type, MSG_CMD_MOVE) == 0) {
        const char* dir_str = doc["direction"] | "stop";
        float speed = doc["speed"] | 0.0f;
        Direction dir = direction_from_string(dir_str);
        // TODO: Pass to gait engine
        (void)dir;
        (void)speed;
        send_ack(MSG_CMD_MOVE, true);
    }
    else if (strcmp(type, MSG_CMD_STAND) == 0) {
        // TODO: Command gait engine to stand
        send_ack(MSG_CMD_STAND, true);
    }
    else if (strcmp(type, MSG_CMD_BALANCE) == 0) {
        balance_enabled = doc["enabled"] | true;
        send_ack(MSG_CMD_BALANCE, true);
    }
    else if (strcmp(type, MSG_CMD_SET_GAIT) == 0) {
        // TODO: Update gait parameters
        send_ack(MSG_CMD_SET_GAIT, true);
    }
    else if (strcmp(type, MSG_CMD_PATROL) == 0) {
        // TODO: Handle patrol start/stop
        send_ack(MSG_CMD_PATROL, true);
    }
    else {
        send_ack(type, false, "unknown_type");
    }
}

// --- Senders ---
void send_pong() {
    JsonDocument doc;
    doc["type"] = MSG_PONG;
    send_json(doc);
}

void send_ack(const char* ref_type, bool ok, const char* error) {
    JsonDocument doc;
    doc["type"] = MSG_ACK;
    doc["ref_type"] = ref_type;
    doc["ok"] = ok;
    if (error) doc["error"] = error;
    send_json(doc);
}

void send_telem_imu() {
    JsonDocument doc;
    doc["type"] = MSG_TELEM_IMU;
    // TODO: Read real IMU values from balance.cpp
    doc["pitch"] = 0.0;
    doc["roll"]  = 0.0;
    doc["yaw"]   = 0.0;
    doc["ax"] = 0.0;
    doc["ay"] = 0.0;
    doc["az"] = 9.81;
    doc["gx"] = 0.0;
    doc["gy"] = 0.0;
    doc["gz"] = 0.0;
    send_json(doc);
}

void send_telem_status() {
    JsonDocument doc;
    doc["type"] = MSG_TELEM_STATUS;
    doc["mode"] = "idle";
    doc["battery_pct"] = -1; // unknown until wired
    doc.createNestedArray("servo_errors");
    send_json(doc);
}

void send_json(const JsonDocument& doc) {
    serializeJson(doc, Serial);
    Serial.println(); // newline delimiter
}

void check_heartbeat() {
    if (connected && (millis() - last_msg_received > HEARTBEAT_TIMEOUT_MS)) {
        connected = false;
        // TODO: Freeze servos (safe mode)
    }
}
