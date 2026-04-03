#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "config.h"
#include "protocol.h"
#include "imu.h"
#include "sonar.h"
#include "servos.h"
#include "gait.h"
#include "calibrate.h"

// WiFi enabled via build flag -DWIFI_ENABLED=1
#ifndef WIFI_ENABLED
#define WIFI_ENABLED 0
#endif

#if WIFI_ENABLED
#include <WiFi.h>
#endif

// --- Forward declarations ---
void handle_message(const JsonDocument& doc);
void send_ack(const char* ref_type, bool ok, const char* error = nullptr);
void send_json(const JsonDocument& doc);
void process_rx(char* buf, size_t& pos, char c, unsigned long now);

// --- State ---
static char serial_rx[MAX_MESSAGE_SIZE];
static size_t serial_rx_pos = 0;
static char tcp_rx[MAX_MESSAGE_SIZE];
static size_t tcp_rx_pos = 0;

static unsigned long last_msg_received = 0;
static bool connected = false;
static bool balance_enabled = false;
static bool low_battery = false;
static bool manual_servo_mode = false;

// Telemetry timers
static unsigned long last_imu = 0;
static unsigned long last_sonar = 0;
static unsigned long last_battery = 0;
static unsigned long last_status = 0;
static unsigned long last_gait = 0;

// I2C mutex for shared bus (also used by calibrate.cpp)
SemaphoreHandle_t i2c_mutex;

// WiFi TCP
#if WIFI_ENABLED
static WiFiServer tcp_server(WIFI_TCP_PORT);
static WiFiClient tcp_client;
static bool wifi_connected = false;
#endif

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
    delay(500);  // brief wait for serial — don't block on ESP32

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

    // Initialize servos
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

    // WiFi setup
#if WIFI_ENABLED
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long wifi_start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifi_start < 10000) {
        delay(250);
    }
    wifi_connected = (WiFi.status() == WL_CONNECTED);
    if (wifi_connected) {
        tcp_server.begin();
        tcp_server.setNoDelay(true);
        // Amber LED = WiFi ready, waiting for TCP client
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50))) {
            sonar_set_rgb(1, LED_BRIGHTNESS, LED_BRIGHTNESS / 2, 0);
            sonar_set_rgb(2, LED_BRIGHTNESS, LED_BRIGHTNESS / 2, 0);
            xSemaphoreGive(i2c_mutex);
        }
    }
#endif

    last_msg_received = millis();

    // Boot status
    JsonDocument doc;
    doc["type"] = "boot";
    doc["imu"] = imu_ok;
    doc["sonar"] = sonar_ok;
    doc["servos"] = servos_ok;
    doc["pins_verified"] = (bool)PINS_VERIFIED;
#if WIFI_ENABLED
    doc["wifi"] = wifi_connected;
    if (wifi_connected) {
        doc["wifi_ip"] = WiFi.localIP().toString();
        doc["tcp_port"] = WIFI_TCP_PORT;
    }
#endif
    send_json(doc);
}

// --- Process a received character into a buffer, dispatch on newline ---
void process_rx(char* buf, size_t& pos, char c, unsigned long now) {
    if (c == '\n') {
        buf[pos] = '\0';
        if (pos > 0) {
            JsonDocument doc;
            if (!deserializeJson(doc, buf)) {
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
        pos = 0;
    } else if (pos < MAX_MESSAGE_SIZE - 1) {
        buf[pos++] = c;
    }
}

// --- Main loop ---
void loop() {
    unsigned long now = millis();

    // Read incoming serial data (NDJSON)
    while (Serial.available()) {
        process_rx(serial_rx, serial_rx_pos, Serial.read(), now);
    }

    // Read incoming TCP data
#if WIFI_ENABLED
    if (wifi_connected) {
        // Accept new client
        if (!tcp_client || !tcp_client.connected()) {
            WiFiClient new_client = tcp_server.available();
            if (new_client) {
                tcp_client = new_client;
                tcp_client.setNoDelay(true);
                tcp_rx_pos = 0;
            }
        }
        // Read from connected client
        if (tcp_client && tcp_client.connected()) {
            while (tcp_client.available()) {
                process_rx(tcp_rx, tcp_rx_pos, tcp_client.read(), now);
            }
        }
    }
#endif

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
        if (mv < BATTERY_LOW_MV && mv > 1000) {
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
#if WIFI_ENABLED
        doc["wifi"] = wifi_connected;
        if (wifi_connected) {
            doc["wifi_ip"] = WiFi.localIP().toString();
            doc["tcp_port"] = WIFI_TCP_PORT;
        }
#endif
        send_json(doc);
        last_status = now;
    }

    // Calibration mode (mutually exclusive with gait)
    if (calibrate_active()) {
        calibrate_update(now);
    }
    // Gait engine (skip during calibration or manual servo mode)
    else if (!manual_servo_mode && now - last_gait >= 1000 / GAIT_UPDATE_HZ) {
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
        manual_servo_mode = false;
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
        manual_servo_mode = false;
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
        manual_servo_mode = true;  // disable gait engine
        uint8_t idx = doc["index"] | 0;
        uint16_t us = doc["pulse_us"] | 1500;
        servo_write_us(idx, us);
        send_ack(MSG_CMD_SERVO, true);
#else
        send_ack(MSG_CMD_SERVO, false, "pins_not_verified");
#endif
    }
    else if (strcmp(type, MSG_CMD_CALIBRATE) == 0) {
        const char* action = doc["action"] | "sweep";
        if (strcmp(action, "stop") == 0) {
            calibrate_stop();
            send_ack(MSG_CMD_CALIBRATE, true);
        } else {
            // Ensure servos are awake
            if (!servos_active()) {
                servos_init();
            }
            gait_set_state(GaitState::STOP);
            CalibrateSweep sw;
            sw.servo = doc["servo"] | 0;
            sw.from_us = doc["from_us"] | (STANDING_POSE[sw.servo] - 100);
            sw.to_us = doc["to_us"] | (STANDING_POSE[sw.servo] + 100);
            sw.step_us = doc["step_us"] | 10;
            sw.dwell_ms = doc["dwell_ms"] | 300;
            sw.tilt_limit = doc["tilt_limit"] | 8.0f;
            calibrate_start(sw);
            send_ack(MSG_CMD_CALIBRATE, true);
        }
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
    // Always send to serial (for USB and debug)
    serializeJson(doc, Serial);
    Serial.println();

    // Also send to TCP client if connected
#if WIFI_ENABLED
    if (tcp_client && tcp_client.connected()) {
        serializeJson(doc, tcp_client);
        tcp_client.println();
    }
#endif
}
