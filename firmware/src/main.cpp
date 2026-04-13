#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "config.h"
#include "protocol.h"
#include "imu.h"
#include "sonar.h"
#include "servos.h"
#include "gait.h"
#include "balance.h"
#include "offsets.h"
#include "command_handlers.h"
// WiFi enabled via build flag -DWIFI_ENABLED=1
#ifndef WIFI_ENABLED
#define WIFI_ENABLED 0
#endif

#if WIFI_ENABLED
#include <WiFi.h>
#endif

// --- Forward declarations ---
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
static bool low_battery = false;
// balance_enabled, manual_servo_mode, test_mode, last_test_cmd owned by command_handlers.cpp

// Telemetry timers
static unsigned long last_imu = 0;
static unsigned long last_sonar = 0;
static unsigned long last_battery = 0;
static unsigned long last_status = 0;
static unsigned long last_gait = 0;

// I2C mutex for shared bus
SemaphoreHandle_t i2c_mutex;

// WiFi TCP
#if WIFI_ENABLED
static WiFiServer tcp_server(WIFI_TCP_PORT);
static WiFiClient tcp_client;
static bool wifi_connected = false;
#endif

// --- Direction helpers ---
// Moved to command_handlers.cpp (Task 4 refactor)
// Direction direction_from_string(const char* str) { ... }
// const char* direction_to_string(Direction dir) { ... }

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

    // Load servo trim offsets from NVS
    offsets_init();

    // Initialize servos
    bool servos_ok = servos_init();

    // Initialize gait engine
    gait_init();

    // Initialize command handler state
    handlers_init();

    // Set LED to indicate boot status
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100))) {
        if (imu_ok && sonar_ok) {
            sonar_set_rgb(1, LED_R_LAVENDER, LED_G_LAVENDER, LED_B_LAVENDER);  // lavender = ready
            sonar_set_rgb(2, LED_R_LAVENDER, LED_G_LAVENDER, LED_B_LAVENDER);
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
            sonar_set_rgb(1, LED_R_LAVENDER, LED_G_LAVENDER, LED_B_LAVENDER);  // lavender = disconnected
            sonar_set_rgb(2, LED_R_LAVENDER, LED_G_LAVENDER, LED_B_LAVENDER);
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
                gait_update_imu(imu.pitch, imu.roll);
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
        doc["balance"] = balance_is_enabled();
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

    // Test mode heartbeat — exit test mode if host goes quiet
    // State is owned by command_handlers; check via accessors.
    // NOTE: actual state reset on timeout is handled inside command_handlers (Task 5).
    // For now, guard the gait engine using the accessor.
    if (handlers_test_mode() && (now - handlers_last_test_cmd() > TEST_HEARTBEAT_MS)) {
        servos_set_frail(false);
        gait_set_state(GaitState::STAND);
        // handlers state (test_mode / manual_servo_mode) will be cleared in Task 5
        // when the timeout logic moves fully into command_handlers.
    }

    // Frail mode duty cycle tracking
    if (servos_update_duty(now)) {
        // In cooldown — LEDs flash amber
        if ((now / 500) % 2 == 0) {
            if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(10))) {
                sonar_set_rgb(1, LED_BRIGHTNESS, LED_BRIGHTNESS / 2, 0);
                sonar_set_rgb(2, LED_BRIGHTNESS, LED_BRIGHTNESS / 2, 0);
                xSemaphoreGive(i2c_mutex);
            }
        }
    }

    // Gait engine (skip during manual servo mode)
    if (!handlers_manual_servo_mode() && now - last_gait >= 1000 / GAIT_UPDATE_HZ) {
        if (!low_battery) {
            gait_update(now);
        }
        last_gait = now;
    }
}

// handle_message() moved to command_handlers.cpp (Task 4 refactor)

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
