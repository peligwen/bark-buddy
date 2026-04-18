// firmware/src/main.cpp
#include <Arduino.h>
#include <ArduinoJson.h>
#include <math.h>
#include "config.h"
#include "protocol.h"
#include "comms.h"
#include "sensor_task.h"
#include "command_handlers.h"
#include "servos.h"
#include "gait.h"
#include "balance.h"
#include "offsets.h"
#include "update_led.h"
#include "buzzer.h"
#include "comms_out.h"
#ifndef WIFI_ENABLED
#define WIFI_ENABLED 0
#endif
#if WIFI_ENABLED
#include <WiFi.h>
#include <ESPmDNS.h>
#endif

// --- Update rainbow LED ---
bool          s_update_led_active   = false;
static unsigned long s_update_led_last_ms = 0;

static void update_rainbow_led_tick(unsigned long now) {
    if (!s_update_led_active) return;
    if (now - s_update_led_last_ms < 30) return;
    s_update_led_last_ms = now;

    // Hue rotation: ~2s per cycle
    float hue = fmodf((float)now / 2000.0f, 1.0f);
    // Sinusoidal brightness: 20-100%, ~3s period
    float brightness = 0.6f + 0.4f * sinf((float)now / 3000.0f * 2.0f * (float)M_PI);
    if (brightness < 0.2f) brightness = 0.2f;
    if (brightness > 1.0f) brightness = 1.0f;

    // HSV to RGB (saturation=1)
    float r = 0, g = 0, b = 0;
    int hi = (int)(hue * 6.0f);
    float f  = hue * 6.0f - hi;
    float bq = brightness * (1.0f - f);
    float bt = brightness * f;
    switch (hi % 6) {
        case 0: r=brightness; g=bt;         b=0;          break;
        case 1: r=bq;         g=brightness; b=0;          break;
        case 2: r=0;          g=brightness; b=bt;         break;
        case 3: r=0;          g=bq;         b=brightness; break;
        case 4: r=bt;         g=0;          b=brightness; break;
        case 5: r=brightness; g=0;          b=bq;         break;
    }
    sensor_led_set(1, (uint8_t)(r * LED_BRIGHTNESS),
                      (uint8_t)(g * LED_BRIGHTNESS),
                      (uint8_t)(b * LED_BRIGHTNESS));
    sensor_led_set(2, (uint8_t)(r * LED_BRIGHTNESS),
                      (uint8_t)(g * LED_BRIGHTNESS),
                      (uint8_t)(b * LED_BRIGHTNESS));
}

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

// --- Boot diagnostic ---
static bool gpio2_at_boot = false;

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
void comms_write_line(const char* line) {
    Serial.println(line);
#if WIFI_ENABLED
    if (tcp_client && tcp_client.connected()) {
        tcp_client.println(line);
    }
#endif
}

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

String get_tcp_client_ip() {
#if WIFI_ENABLED
    if (tcp_client && tcp_client.connected()) {
        return tcp_client.remoteIP().toString();
    }
#endif
    return String();
}

// --- Setup ---
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(100);

    // GPIO 2 boot diagnostic — captured BEFORE the sensor task or anything else
    // configures pins. Shared with GPIO usage elsewhere; informational only.
    pinMode(2, INPUT);
    gpio2_at_boot = digitalRead(2);

    // Outbound JSON queue must be ready before the sensor task starts, since
    // the sensor task may call comms_emit_json_from_task() immediately.
    comms_out_init();

    // Sensor task starts I2C, probes IMU + sonar, sets boot LED.
    // Blocks until first init pass completes (<=1s).
    sensor_task_start();
#ifndef MOCK_FIRMWARE
    buzzer_init();
#endif

    // Boot rainbow — runs briefly on every boot, confirms firmware is alive after a flash
    s_update_led_active = true;
    unsigned long boot_rainbow_start = millis();
    while (millis() - boot_rainbow_start < 2000) {
        update_rainbow_led_tick(millis());
        delay(10);
    }
    s_update_led_active = false;

    // Dog boots disengaged. Servos stay detached until the operator engages.
    offsets_init();
    gait_init(millis());
    handlers_init();

#if WIFI_ENABLED
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    // No blocking wait — loop() handles connect and reconnect
#endif

    // Reset heartbeat clock after boot. If no host connects within HEARTBEAT_TIMEOUT_MS,
    // the heartbeat watchdog fires and servos_detach_all() keeps the dog safe.
    last_msg_received = millis();

    // Boot message
    SensorSnapshot snap;
    sensor_snapshot_get(snap);
    JsonDocument doc;
    doc["type"]           = "boot";
    doc["imu"]            = snap.imu_ok;
    doc["sonar"]          = snap.sonar_ok;
    doc["pins_verified"]  = (bool)PINS_VERIFIED;
    doc["fw_version"]     = FW_VERSION;
    doc["fw_build"]       = FW_BUILD_TIMESTAMP;
    doc["gpio2_at_boot"]  = gpio2_at_boot;
    send_json(doc);
    broadcast_servo_pins();
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
            if (MDNS.begin("mechdog")) {
                MDNS.addService("_mechdog", "_tcp", WIFI_TCP_PORT);
                MDNS.addServiceTxt("_mechdog", "_tcp", "fw_version", FW_VERSION);
                MDNS.addServiceTxt("_mechdog", "_tcp", "board", "esp32-d0wd");
            }
            sensor_led_set(1, LED_BRIGHTNESS, LED_BRIGHTNESS / 2, 0);  // amber = waiting
            sensor_led_set(2, LED_BRIGHTNESS, LED_BRIGHTNESS / 2, 0);
        }
        if (!now_wifi && wifi_connected) {
            wifi_connected = false;
            tcp_client.stop();
            MDNS.end();
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

    // Heartbeat watchdog — if the host goes quiet, detach servos + stop gait.
    // If host goes quiet during an engage ramp, cut power immediately rather than
    // completing the ramp. The 10s heartbeat timeout is much longer than the 2s
    // engage ramp, so this only fires if the host is genuinely gone.
    if (connected && (now - last_msg_received > HEARTBEAT_TIMEOUT_MS)) {
        connected = false;
        JsonDocument hb_evt;
        hb_evt["type"]              = MSG_TELEM_EVENT;
        hb_evt["event"]             = "heartbeat_detach";
        hb_evt["t"]                 = (uint32_t)now;
        hb_evt["ms_since_last_msg"] = (uint32_t)(now - last_msg_received);
        send_json(hb_evt);
        servos_detach_all();
        gait_set_state(GaitState::STOP);
        sensor_led_set(1, LED_R_LAVENDER, LED_G_LAVENDER, LED_B_LAVENDER);
        sensor_led_set(2, LED_R_LAVENDER, LED_G_LAVENDER, LED_B_LAVENDER);
    }

    // Battery check
    if (now - last_battery >= 1000 / TELEM_BATTERY_HZ) {
        int   raw     = analogRead(BATTERY_ADC_PIN);
        float voltage = (raw / 4095.0f) * 3.3f * BATTERY_DIVIDER;
        int   mv      = (int)(voltage * 1000);
        if (mv < BATTERY_LOW_MV && mv > 1000 && !servos_battery_cutoff()) {
            servos_set_battery_cutoff();  // latches AND detaches
            gait_set_state(GaitState::STOP);
            JsonDocument lb_evt;
            lb_evt["type"]  = MSG_TELEM_EVENT;
            lb_evt["event"] = "battery_cutoff_detach";
            lb_evt["t"]     = (uint32_t)now;
            lb_evt["mv"]    = mv;
            send_json(lb_evt);
        }
        if (connected) {
            JsonDocument doc;
            doc["type"]       = MSG_TELEM_BATTERY;
            doc["voltage_mv"] = mv;
            doc["pct"]        = constrain((mv - 6000) * 100 / 2400, 0, 100);
            doc["low"]        = servos_battery_cutoff();
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
        doc["type"]                   = MSG_TELEM_STATUS;
        doc["engaged"]                = servos_engaged();
        doc["ramping"]                = servos_is_ramping();
        doc["balance"]                = balance_is_enabled();
        doc["battery_cutoff"]         = servos_battery_cutoff();
        doc["ms_since_last_host_msg"] = (uint32_t)(now - last_msg_received);
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

    // Drain sensor-task outbound JSON queue (serialized to buf by sender, written here).
    comms_out_drain();

    // Update rainbow LED tick
    update_rainbow_led_tick(now);

    // Drive engage/disengage ramp. Emit event on completion.
    {
        RampResult ramp = servos_ramp_tick(now);
        if (ramp != RampResult::NONE) {
            JsonDocument evt;
            evt["type"]  = MSG_TELEM_EVENT;
            evt["t"]     = (uint32_t)now;
            evt["event"] = (ramp == RampResult::ENGAGE_COMPLETE) ? "engage_complete" : "disengage_complete";
            send_json(evt);
        }
    }

    // Gait tick — only when not ramping. gait_update() guards on servos_engaged() internally.
    if (!servos_is_ramping() && now - last_gait >= 1000 / GAIT_UPDATE_HZ) {
        gait_update(now);
        last_gait = now;
    }
}
