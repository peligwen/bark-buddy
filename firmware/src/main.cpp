// firmware/src/main.cpp
#include <Arduino.h>
#include <ArduinoJson.h>
#include "config.h"
#include "protocol.h"
#include "comms.h"
#include "sensor_task.h"
#include "command_handlers.h"
#include "servos.h"
#include "gait.h"
#include "balance.h"
#include "offsets.h"
#ifndef WIFI_ENABLED
#define WIFI_ENABLED 0
#endif
#if WIFI_ENABLED
#include <WiFi.h>
#endif

// --- RX buffers ---
static char   serial_rx[MAX_MESSAGE_SIZE];
static size_t serial_rx_pos = 0;
#if WIFI_ENABLED
static char   tcp_rx[MAX_MESSAGE_SIZE];
static size_t tcp_rx_pos = 0;
#endif

// --- Connection state ---
static unsigned long last_msg_received = 0;
static bool          connected         = false;
static bool          low_battery       = false;

// --- Telemetry timers ---
static unsigned long last_imu     = 0;
static unsigned long last_sonar   = 0;
static unsigned long last_battery = 0;
static unsigned long last_status  = 0;
static unsigned long last_gait    = 0;

// --- WiFi / TCP ---
#if WIFI_ENABLED
static WiFiServer tcp_server(WIFI_TCP_PORT);
static WiFiClient tcp_client;
static bool       wifi_connected = false;
#endif

// --- Process received character; dispatch on newline ---
void process_rx(char* buf, size_t& pos, char c, unsigned long now) {
    if (c == '\n') {
        buf[pos] = '\0';
        if (pos > 0) {
            JsonDocument doc;
            if (!deserializeJson(doc, buf)) {
                last_msg_received = now;
                if (!connected) {
                    connected = true;
                    sensor_led_set(1, 0, LED_BRIGHTNESS, 0);  // green = connected
                    sensor_led_set(2, 0, LED_BRIGHTNESS, 0);
                }
                handle_message(doc);
            }
        }
        pos = 0;
    } else if (pos < MAX_MESSAGE_SIZE - 1) {
        buf[pos++] = c;
    }
}

// --- Send helpers (declared in comms.h) ---
void send_json(const JsonDocument& doc) {
    serializeJson(doc, Serial);
    Serial.println();
#if WIFI_ENABLED
    if (tcp_client && tcp_client.connected()) {
        serializeJson(doc, tcp_client);
        tcp_client.println();
    }
#endif
}

void send_ack(const char* ref_type, bool ok, const char* error) {
    JsonDocument doc;
    doc["type"]     = MSG_ACK;
    doc["ref_type"] = ref_type;
    doc["ok"]       = ok;
    if (error) doc["error"] = error;
    send_json(doc);
}

// --- Setup ---
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(100);

    // Sensor task starts I2C, probes IMU + sonar, sets boot LED.
    // Blocks until first init pass completes (≤1s).
    sensor_task_start();

    offsets_init();
    bool servos_ok = servos_attach_at(REST_POSE);
    delay(BOOT_SETTLE_MS);
    servos_ramp_to(STANDING_POSE, SOFTSTART_DURATION_MS, SOFTSTART_STEPS);
    gait_init(millis());
    lifecycle_boot_complete(millis());
    handlers_init();

#if WIFI_ENABLED
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    // No blocking wait — loop() handles connect and reconnect
#endif

    // Reset heartbeat clock after boot. If no host connects within HEARTBEAT_TIMEOUT_MS (5s),
    // lifecycle_heartbeat_lost fires and the dog will ramp back to rest pose and detach servos.
    // This is intentional safe-fail behavior.
    last_msg_received = millis();

    // Boot message — sensor init results are ready from the snapshot
    SensorSnapshot snap;
    sensor_snapshot_get(snap);
    JsonDocument doc;
    doc["type"]          = "boot";
    doc["imu"]           = snap.imu_ok;
    doc["sonar"]         = snap.sonar_ok;
    doc["servos"]        = servos_ok;
    doc["pins_verified"] = (bool)PINS_VERIFIED;
    doc["fw_version"]   = FW_VERSION;
    doc["fw_build"]     = FW_BUILD_TIMESTAMP;
    send_json(doc);
}

// --- Main loop ---
void loop() {
    unsigned long now = millis();

    // WiFi connect / reconnect
#if WIFI_ENABLED
    {
        bool now_wifi = (WiFi.status() == WL_CONNECTED);
        if (now_wifi && !wifi_connected) {
            wifi_connected = true;
            tcp_server.begin();
            tcp_server.setNoDelay(true);
            sensor_led_set(1, LED_BRIGHTNESS, LED_BRIGHTNESS / 2, 0);  // amber = waiting
            sensor_led_set(2, LED_BRIGHTNESS, LED_BRIGHTNESS / 2, 0);
        }
        if (!now_wifi && wifi_connected) {
            wifi_connected = false;
            tcp_client.stop();
            WiFi.reconnect();
        }
    }
#endif

    // Read serial
    while (Serial.available()) {
        process_rx(serial_rx, serial_rx_pos, Serial.read(), now);
    }

    // Read TCP
#if WIFI_ENABLED
    if (wifi_connected) {
        if (!tcp_client || !tcp_client.connected()) {
            WiFiClient c = tcp_server.available();
            if (c) { tcp_client = c; tcp_client.setNoDelay(true); tcp_rx_pos = 0; }
        }
        if (tcp_client && tcp_client.connected()) {
            while (tcp_client.available()) {
                process_rx(tcp_rx, tcp_rx_pos, tcp_client.read(), now);
            }
        }
    }
#endif

    // Heartbeat watchdog
    if (connected && (now - last_msg_received > HEARTBEAT_TIMEOUT_MS)) {
        connected = false;
        lifecycle_heartbeat_lost(now);
        sensor_led_set(1, LED_R_LAVENDER, LED_G_LAVENDER, LED_B_LAVENDER);
        sensor_led_set(2, LED_R_LAVENDER, LED_G_LAVENDER, LED_B_LAVENDER);
    }

    // Battery check
    if (now - last_battery >= 1000 / TELEM_BATTERY_HZ) {
        int   raw     = analogRead(BATTERY_ADC_PIN);
        float voltage = (raw / 4095.0f) * 3.3f * BATTERY_DIVIDER;
        int   mv      = (int)(voltage * 1000);
        if (mv < BATTERY_LOW_MV && mv > 1000 && !low_battery) {
            low_battery = true;
            servos_detach_all();
            gait_set_state(GaitState::STOP);
        }
        if (connected) {
            JsonDocument doc;
            doc["type"]       = MSG_TELEM_BATTERY;
            doc["voltage_mv"] = mv;
            doc["pct"]        = constrain((mv - 6000) * 100 / 2400, 0, 100);
            doc["low"]        = low_battery;
            send_json(doc);
        }
        last_battery = now;
    }

    // Read snapshot once per loop tick — used for IMU telem, sonar telem, and gait
    SensorSnapshot snap;
    sensor_snapshot_get(snap);

    // IMU streaming + feed gait balance
    if (connected && now - last_imu >= 1000 / TELEM_IMU_HZ) {
        if (snap.imu_ok) {
            gait_update_imu(snap.pitch, snap.roll);
            JsonDocument doc;
            doc["type"]  = MSG_TELEM_IMU;
            doc["pitch"] = round(snap.pitch * 10) / 10.0;
            doc["roll"]  = round(snap.roll  * 10) / 10.0;
            doc["yaw"]   = round(snap.yaw   * 10) / 10.0;
            doc["ax"]    = round(snap.ax * 100) / 100.0;
            doc["ay"]    = round(snap.ay * 100) / 100.0;
            doc["az"]    = round(snap.az * 100) / 100.0;
            doc["gx"]    = round(snap.gx * 10) / 10.0;
            doc["gy"]    = round(snap.gy * 10) / 10.0;
            doc["gz"]    = round(snap.gz * 10) / 10.0;
            send_json(doc);
        }
        last_imu = now;
    }

    // Sonar streaming
    if (connected && now - last_sonar >= 1000 / TELEM_SONAR_HZ) {
        JsonDocument doc;
        doc["type"]        = MSG_TELEM_SONAR;
        doc["distance_mm"] = snap.sonar_mm;
        send_json(doc);
        last_sonar = now;
    }

    // Status streaming
    if (connected && now - last_status >= 1000 / TELEM_STATUS_HZ) {
        JsonDocument doc;
        doc["type"]        = MSG_TELEM_STATUS;
        doc["mode"]        = "idle";
        doc["lifecycle"]   = lifecycle_state_name();
        doc["balance"]     = balance_is_enabled();
        doc["servos"]      = servos_active();
        doc["low_battery"] = low_battery;
#if WIFI_ENABLED
        doc["wifi"] = wifi_connected;
        if (wifi_connected) {
            doc["wifi_ip"]  = WiFi.localIP().toString();
            doc["tcp_port"] = WIFI_TCP_PORT;
        }
#endif
        send_json(doc);
        last_status = now;
    }

    // Test mode heartbeat — exit test mode if host goes quiet
    handlers_check_timeout(now);

    // Frail mode duty cycle
    if (servos_update_duty(now)) {
        if ((now / 500) % 2 == 0) {
            sensor_led_set(1, LED_BRIGHTNESS, LED_BRIGHTNESS / 2, 0);  // amber flash
            sensor_led_set(2, LED_BRIGHTNESS, LED_BRIGHTNESS / 2, 0);
        }
    }

    // Gait tick (skip during manual servo mode)
    if (!handlers_manual_servo_mode() && now - last_gait >= 1000 / GAIT_UPDATE_HZ) {
        lifecycle_update(now);
        LifecycleState lc = lifecycle_current();
        if (!low_battery && (lc == LifecycleState::ACTIVE || lc == LifecycleState::IDLE)) {
            gait_update(now);
        }
        last_gait = now;
    }
}
