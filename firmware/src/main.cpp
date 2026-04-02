#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "config.h"
#include "protocol.h"
#include "imu.h"
#include "sonar.h"
#include "servos.h"
#include "gait.h"

// --- Forward declarations ---
void handle_message(const JsonDocument& doc);
void send_ack(const char* ref_type, bool ok, const char* error = nullptr);
void send_json(const JsonDocument& doc);

// --- State ---
static char rx_buffer[MAX_MESSAGE_SIZE];
static size_t rx_pos = 0;
static unsigned long last_msg_received = 0;
static bool connected = false;
static bool balance_enabled = false;
static bool low_battery = false;

// Telemetry timers
static unsigned long last_imu = 0;
static unsigned long last_sonar = 0;
static unsigned long last_battery = 0;
static unsigned long last_status = 0;
static unsigned long last_gait = 0;

// I2C mutex for shared bus
static SemaphoreHandle_t i2c_mutex;

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

    // I2C mutex
    i2c_mutex = xSemaphoreCreateMutex();

    // Initialize I2C bus
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);

    // Initialize sensors
    bool imu_ok = false, sonar_ok = false;
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100))) {
        imu_ok = imu_init(Wire);
        sonar_ok = sonar_init(Wire);
        xSemaphoreGive(i2c_mutex);
    }

    // Initialize servos (will fail gracefully if PINS_VERIFIED=0)
    bool servos_ok = servos_init();

    // Initialize gait engine
    gait_init();

    // Set LED to indicate boot status
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100))) {
        if (imu_ok && sonar_ok) {
            sonar_set_rgb(1, 0, 0, LED_BRIGHTNESS);  // blue = ready
            sonar_set_rgb(2, 0, 0, LED_BRIGHTNESS);
        } else {
            sonar_set_rgb(1, LED_BRIGHTNESS, 0, 0);   // red = error
            sonar_set_rgb(2, LED_BRIGHTNESS, 0, 0);
        }
        xSemaphoreGive(i2c_mutex);
    }

    last_msg_received = millis();

    // Boot status
    JsonDocument doc;
    doc["type"] = "boot";
    doc["imu"] = imu_ok;
    doc["sonar"] = sonar_ok;
    doc["servos"] = servos_ok;
    doc["pins_verified"] = (bool)PINS_VERIFIED;
    send_json(doc);
}

// --- Main loop ---
void loop() {
    unsigned long now = millis();

    // Read incoming serial data (NDJSON)
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            rx_buffer[rx_pos] = '\0';
            if (rx_pos > 0) {
                JsonDocument doc;
                if (!deserializeJson(doc, rx_buffer)) {
                    last_msg_received = now;
                    if (!connected) {
                        connected = true;
                        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50))) {
                            sonar_set_rgb(1, 0, LED_BRIGHTNESS, 0);  // green = connected
                            sonar_set_rgb(2, 0, LED_BRIGHTNESS, 0);
                            xSemaphoreGive(i2c_mutex);
                        }
                    }
                    handle_message(doc);
                }
            }
            rx_pos = 0;
        } else if (rx_pos < MAX_MESSAGE_SIZE - 1) {
            rx_buffer[rx_pos++] = c;
        }
    }

    // Heartbeat timeout
    if (connected && (now - last_msg_received > HEARTBEAT_TIMEOUT_MS)) {
        connected = false;
        gait_set_state(GaitState::STOP);
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50))) {
            sonar_set_rgb(1, 0, 0, LED_BRIGHTNESS);  // blue = disconnected
            sonar_set_rgb(2, 0, 0, LED_BRIGHTNESS);
            xSemaphoreGive(i2c_mutex);
        }
    }

    // Battery check
    if (now - last_battery >= 1000 / TELEM_BATTERY_HZ) {
        int raw = analogRead(BATTERY_ADC_PIN);
        float voltage = (raw / 4095.0f) * 3.3f * BATTERY_DIVIDER;
        int mv = (int)(voltage * 1000);
        if (mv < BATTERY_LOW_MV && mv > 1000) {  // > 1000 to ignore unconnected ADC
            if (!low_battery) {
                low_battery = true;
                servos_detach_all();
                gait_set_state(GaitState::STOP);
            }
        }
        if (connected) {
            JsonDocument doc;
            doc["type"] = MSG_TELEM_BATTERY;
            doc["voltage_mv"] = mv;
            doc["pct"] = constrain((mv - 6000) * 100 / 2400, 0, 100);
            doc["low"] = low_battery;
            send_json(doc);
        }
        last_battery = now;
    }

    // IMU streaming
    if (connected && now - last_imu >= 1000 / TELEM_IMU_HZ) {
        IMUData imu;
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(10))) {
            bool ok = imu_read(imu);
            xSemaphoreGive(i2c_mutex);
            if (ok) {
                JsonDocument doc;
                doc["type"] = MSG_TELEM_IMU;
                doc["pitch"] = round(imu.pitch * 10) / 10.0;
                doc["roll"] = round(imu.roll * 10) / 10.0;
                doc["yaw"] = round(imu.yaw * 10) / 10.0;
                doc["ax"] = round(imu.ax * 100) / 100.0;
                doc["ay"] = round(imu.ay * 100) / 100.0;
                doc["az"] = round(imu.az * 100) / 100.0;
                doc["gx"] = round(imu.gx * 10) / 10.0;
                doc["gy"] = round(imu.gy * 10) / 10.0;
                doc["gz"] = round(imu.gz * 10) / 10.0;
                send_json(doc);
            }
        }
        last_imu = now;
    }

    // Sonar streaming
    if (connected && now - last_sonar >= 1000 / TELEM_SONAR_HZ) {
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(10))) {
            uint16_t dist = sonar_read_mm();
            xSemaphoreGive(i2c_mutex);
            JsonDocument doc;
            doc["type"] = MSG_TELEM_SONAR;
            doc["distance_mm"] = dist;
            send_json(doc);
        }
        last_sonar = now;
    }

    // Status
    if (connected && now - last_status >= 1000 / TELEM_STATUS_HZ) {
        JsonDocument doc;
        doc["type"] = MSG_TELEM_STATUS;
        doc["mode"] = "idle";
        doc["balance"] = balance_enabled;
        doc["servos"] = servos_active();
        doc["low_battery"] = low_battery;
        send_json(doc);
        last_status = now;
    }

    // Gait engine
    if (now - last_gait >= 1000 / GAIT_UPDATE_HZ) {
        if (!low_battery) {
            gait_update(now);
        }
        last_gait = now;
    }
}

// --- Message handler ---
void handle_message(const JsonDocument& doc) {
    const char* type = doc["type"];
    if (!type) return;

    if (strcmp(type, MSG_PING) == 0) {
        JsonDocument resp;
        resp["type"] = MSG_PONG;
        send_json(resp);
    }
    else if (strcmp(type, MSG_CMD_MOVE) == 0) {
        const char* dir_str = doc["direction"] | "stop";
        float spd = doc["speed"] | 1.0f;
        Direction dir = direction_from_string(dir_str);
        switch (dir) {
            case Direction::FORWARD:  gait_set_state(GaitState::WALK_FORWARD, spd); break;
            case Direction::BACKWARD: gait_set_state(GaitState::WALK_BACKWARD, spd); break;
            case Direction::LEFT:     gait_set_state(GaitState::TURN_LEFT, spd); break;
            case Direction::RIGHT:    gait_set_state(GaitState::TURN_RIGHT, spd); break;
            case Direction::STOP:     gait_set_state(GaitState::STOP); break;
        }
        send_ack(MSG_CMD_MOVE, true);
    }
    else if (strcmp(type, MSG_CMD_STAND) == 0) {
        gait_set_state(GaitState::STAND);
        send_ack(MSG_CMD_STAND, true);
    }
    else if (strcmp(type, MSG_CMD_BALANCE) == 0) {
        balance_enabled = doc["enabled"] | true;
        send_ack(MSG_CMD_BALANCE, true);
    }
    else if (strcmp(type, MSG_CMD_LED) == 0) {
        uint8_t led = doc["led"] | 1;
        uint8_t r = doc["r"] | 0;
        uint8_t g = doc["g"] | 0;
        uint8_t b = doc["b"] | 0;
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50))) {
            sonar_set_rgb(led, r, g, b);
            xSemaphoreGive(i2c_mutex);
        }
        send_ack(MSG_CMD_LED, true);
    }
    else if (strcmp(type, MSG_CMD_SERVO) == 0) {
#if PINS_VERIFIED
        uint8_t idx = doc["index"] | 0;
        uint16_t us = doc["pulse_us"] | 1500;
        servo_write_us(idx, us);
        send_ack(MSG_CMD_SERVO, true);
#else
        send_ack(MSG_CMD_SERVO, false, "pins_not_verified");
#endif
    }
    else {
        send_ack(type, false, "unknown_type");
    }
}

// --- Senders ---
void send_ack(const char* ref_type, bool ok, const char* error) {
    JsonDocument doc;
    doc["type"] = MSG_ACK;
    doc["ref_type"] = ref_type;
    doc["ok"] = ok;
    if (error) doc["error"] = error;
    send_json(doc);
}

void send_json(const JsonDocument& doc) {
    serializeJson(doc, Serial);
    Serial.println();
}
