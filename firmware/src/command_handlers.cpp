// firmware/src/command_handlers.cpp
#include "command_handlers.h"
#include "comms.h"
#include "protocol.h"
#include "sensor_task.h"
#include "config.h"
#include "gait.h"
#include "servos.h"
#include "balance.h"
#include "offsets.h"
#include "update_led.h"
#include "buzzer.h"
#include "gpio_aux.h"
#include "pin_registry.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <string.h>
#if WIFI_ENABLED
#include <HTTPClient.h>
#include <Update.h>
#include <mbedtls/sha256.h>
#endif

// --- Direction helpers ---
Direction direction_from_string(const char* str) {
    if (strcmp(str, "forward")  == 0) return Direction::FORWARD;
    if (strcmp(str, "backward") == 0) return Direction::BACKWARD;
    if (strcmp(str, "left")     == 0) return Direction::LEFT;
    if (strcmp(str, "right")    == 0) return Direction::RIGHT;
    return Direction::STOP;
}

// --- Gate helper: motion commands require engaged servos ---
static bool require_engaged(const char* cmd_type) {
    if (servos_battery_cutoff()) {
        send_ack(cmd_type, false, "battery_cutoff");
        return false;
    }
    if (!servos_engaged()) {
        send_ack(cmd_type, false, "not_engaged");
        return false;
    }
    if (servos_is_ramping()) {
        send_ack(cmd_type, false, "ramping");
        return false;
    }
    return true;
}

// --- Broadcast helpers ---

void broadcast_servo_pins() {
    JsonDocument resp;
    resp["type"] = MSG_TELEM_SERVO_PINS;
    JsonArray arr = resp["pins"].to<JsonArray>();
    for (int i = 0; i < 8; i++) arr.add(SERVO_PINS[i]);
    send_json(resp);
}

// --- Handlers ---

static void handle_ping(const JsonDocument&) {
    JsonDocument resp;
    resp["type"] = MSG_PONG;
    send_json(resp);
}

static void handle_cmd_engage(const JsonDocument& doc) {
    bool enable = doc["enabled"] | false;
    if (enable) {
        if (servos_battery_cutoff()) {
            send_ack(MSG_CMD_ENGAGE, false, "battery_cutoff");
            return;
        }
        if (servos_is_ramping()) {
            send_ack(MSG_CMD_ENGAGE, false, "ramping");
            return;
        }
        if (servos_engaged()) {
            send_ack(MSG_CMD_ENGAGE, true);  // already engaged, no-op
            return;
        }
        if (!servos_engage_start()) {
            send_ack(MSG_CMD_ENGAGE, false, "engage_failed");
            return;
        }
        JsonDocument evt;
        evt["type"]  = MSG_TELEM_EVENT;
        evt["event"] = "engage_start";
        evt["t"]     = (uint32_t)millis();
        send_json(evt);
        send_ack(MSG_CMD_ENGAGE, true);
    } else {
        if (!servos_engaged() && !servos_is_ramping()) {
            send_ack(MSG_CMD_ENGAGE, true);  // already disengaged, no-op
            return;
        }
        gait_set_state(GaitState::STOP);
        servos_disengage_start();
        JsonDocument evt;
        evt["type"]  = MSG_TELEM_EVENT;
        evt["event"] = "disengage_start";
        evt["t"]     = (uint32_t)millis();
        send_json(evt);
        send_ack(MSG_CMD_ENGAGE, true);
    }
}

static void handle_cmd_move(const JsonDocument& doc) {
    if (!require_engaged(MSG_CMD_MOVE)) return;
    const char* dir_str = doc["direction"] | "stop";
    float spd = doc["speed"] | 1.0f;
    Direction dir = direction_from_string(dir_str);
    switch (dir) {
        case Direction::FORWARD:  gait_set_state(GaitState::WALK_FORWARD,  spd); break;
        case Direction::BACKWARD: gait_set_state(GaitState::WALK_BACKWARD, spd); break;
        case Direction::LEFT:     gait_set_state(GaitState::TURN_LEFT,     spd); break;
        case Direction::RIGHT:    gait_set_state(GaitState::TURN_RIGHT,    spd); break;
        case Direction::STOP:     gait_set_state(GaitState::STOP);               break;
    }
    send_ack(MSG_CMD_MOVE, true);
}

static void handle_cmd_stand(const JsonDocument&) {
    if (!require_engaged(MSG_CMD_STAND)) return;
    gait_set_state(GaitState::STAND);
    send_ack(MSG_CMD_STAND, true);
}

static void handle_cmd_balance(const JsonDocument& doc) {
    if (!require_engaged(MSG_CMD_BALANCE)) return;
    bool enabled = doc["enabled"] | true;
    balance_enable(enabled);
    if (!enabled) balance_reset();
    send_ack(MSG_CMD_BALANCE, true);
}

static void handle_cmd_transform(const JsonDocument& doc) {
    if (!require_engaged(MSG_CMD_TRANSFORM)) return;
    BodyPose pose;
    pose.dx    = doc["x"]     | 0.0f;
    pose.dy    = doc["y"]     | 0.0f;
    pose.dz    = doc["z"]     | 0.0f;
    pose.roll  = doc["roll"]  | 0.0f;
    pose.pitch = doc["pitch"] | 0.0f;
    pose.yaw   = doc["yaw"]   | 0.0f;
    uint16_t ms = doc["ms"]   | 100;
    gait_set_body_transform(pose, ms);
    send_ack(MSG_CMD_TRANSFORM, true);
}

static void handle_cmd_gait_params(const JsonDocument& doc) {
    if (!require_engaged(MSG_CMD_GAIT_PARAMS)) return;
    GaitConfig cfg;
    cfg.stride_height_mm = doc["stride_height"] | GAIT_STRIDE_HEIGHT_MM;
    cfg.stride_length_mm = doc["stride_length"] | GAIT_STRIDE_LENGTH_MM;
    cfg.frequency_hz     = doc["frequency"]     | GAIT_FREQUENCY_HZ;
    gait_set_config(cfg);
    send_ack(MSG_CMD_GAIT_PARAMS, true);
}

static void handle_cmd_led(const JsonDocument& doc) {
    uint8_t led = doc["led"] | 1;
    if (led == 0) {
        send_ack(MSG_CMD_LED, false);
        return;
    }
    uint8_t r = doc["r"] | 0;
    uint8_t g = doc["g"] | 0;
    uint8_t b = doc["b"] | 0;
    sensor_led_set(led, r, g, b);
    send_ack(MSG_CMD_LED, true);
}

static void handle_cmd_servo(const JsonDocument& doc) {
    if (!require_engaged(MSG_CMD_SERVO)) return;

    uint8_t  idx = doc["index"]    | 0;
    uint16_t us  = doc["pulse_us"] | 1500;

    if (idx >= 11) {
        send_ack(MSG_CMD_SERVO, false, "bad_index");
        return;
    }
#if !AUX_SERVOS_ENABLED
    if (idx >= 8) {
        send_ack(MSG_CMD_SERVO, false, "aux_disabled");
        return;
    }
#endif

#if AUX_SERVOS_ENABLED
    if (idx >= 8 && idx <= 10) {
        aux_servo_write_us(idx - 8, us);
        JsonDocument resp;
        resp["type"]      = MSG_ACK;
        resp["ref_type"]  = MSG_CMD_SERVO;
        resp["ok"]        = true;
        resp["index"]     = idx;
        resp["actual_us"] = aux_servo_read_us(idx - 8);
        send_json(resp);
        return;
    }
#endif

    gait_pause();  // prevent gait from fighting manual writes
    bool written = servo_write_us(idx, us);

    JsonDocument resp;
    resp["type"]      = MSG_ACK;
    resp["ref_type"]  = MSG_CMD_SERVO;
    resp["ok"]        = true;
    resp["index"]     = idx;
    resp["actual_us"] = servo_read_us(idx);
    resp["written"]   = written;
    send_json(resp);
}

static void handle_cmd_i2c(const JsonDocument& doc) {
    const char* op_str = doc["op"] | "";
    if (op_str[0] == '\0') {
        send_ack(MSG_CMD_I2C, false, "missing op");
        return;
    }
    uint8_t bus = doc["bus"] | 1;

    // SCAN stalls the sensor task ~130ms — avoid during active balance/gait.
    I2cOp op;
    if      (strcmp(op_str, "scan")  == 0) op = I2cOp::SCAN;
    else if (strcmp(op_str, "read")  == 0) op = I2cOp::READ;
    else if (strcmp(op_str, "write") == 0) op = I2cOp::WRITE;
    else {
        send_ack(MSG_CMD_I2C, false, "unknown_op");
        return;
    }

    if (op == I2cOp::SCAN && (balance_is_enabled() || gait_current_state() != GaitState::STOP)) {
        send_ack(MSG_CMD_I2C, false, "scan_blocked_during_motion");
        return;
    }

    I2cCmd cmd = {};
    cmd.op   = op;
    cmd.bus  = bus;
    cmd.addr = doc["addr"] | 0;
    cmd.reg  = doc["reg"]  | 0;
    cmd.val  = doc["val"]  | 0;
    cmd.len  = doc["len"]  | 1;
    if (cmd.len == 0 || cmd.len > 32) cmd.len = 1;

    I2cResult res;
    if (!sensor_i2c_op(cmd, res)) {
        send_ack(MSG_CMD_I2C, false, "timeout");
        return;
    }

    JsonDocument resp;
    resp["type"] = MSG_TELEM_I2C;
    resp["bus"]  = bus;
    resp["op"]   = op_str;
    if (op == I2cOp::SCAN) {
        JsonArray arr = resp["addrs"].to<JsonArray>();
        for (uint8_t i = 0; i < res.addr_count; i++) arr.add(res.addrs[i]);
    } else if (op == I2cOp::READ) {
        resp["addr"] = cmd.addr;
        resp["reg"]  = cmd.reg;
        JsonArray arr = resp["data"].to<JsonArray>();
        for (uint8_t i = 0; i < res.data_len; i++) arr.add(res.data[i]);
    } else {
        resp["ok"] = res.ok;
    }
    send_json(resp);
}

static void handle_cmd_offset(const JsonDocument& doc) {
    // No engagement required — offsets are NVS I/O only
    if (!doc["offset_us"].isNull()) {
        uint8_t idx = doc["index"] | 255;
        if (idx >= 8) {
            send_ack(MSG_CMD_OFFSET, false, "bad_index");
            return;
        }
        int16_t val = doc["offset_us"] | 0;
        offset_set(idx, val);
        offsets_save();
    }
    // Always reply with all 8 offsets
    JsonDocument resp;
    resp["type"]     = MSG_ACK;
    resp["ref_type"] = MSG_CMD_OFFSET;
    resp["ok"]       = true;
    JsonArray arr = resp["offsets"].to<JsonArray>();
    for (int i = 0; i < 8; i++) arr.add(offset_get(i));
    send_json(resp);
}

static void handle_cmd_ota_update(const JsonDocument& doc) {
#if !WIFI_ENABLED
    send_ack(MSG_CMD_OTA_UPDATE, false, "wifi_disabled");
    return;
#else
    const char* url      = doc["url"]    | "";
    const char* expected_sha256 = doc["sha256"] | "";
    if (!url || url[0] == '\0') {
        send_ack(MSG_CMD_OTA_UPDATE, false, "missing_url");
        return;
    }
    // Only allow OTA downloads from the connected TCP client.
    // Note: IPv6 bracket notation (e.g. http://[::1]/path) is not supported.
    {
        String client_ip = get_tcp_client_ip();
        if (client_ip.isEmpty()) {
            send_ack(MSG_CMD_OTA_UPDATE, false, "url_not_allowed");
            return;
        }
        const char* after_scheme = strstr(url, "://");
        if (!after_scheme) {
            send_ack(MSG_CMD_OTA_UPDATE, false, "url_not_allowed");
            return;
        }
        after_scheme += 3; // skip past "://"
        // Reject URLs containing '@' in the authority (userinfo bypass prevention)
        const char* auth_end = strchr(after_scheme, '/');
        size_t auth_len = auth_end ? (size_t)(auth_end - after_scheme) : strlen(after_scheme);
        if (memchr(after_scheme, '@', auth_len) != nullptr) {
            send_ack(MSG_CMD_OTA_UPDATE, false, "url_not_allowed");
            return;
        }
        const char* end = after_scheme;
        while (*end && *end != ':' && *end != '/') end++;
        String url_host(after_scheme, (size_t)(end - after_scheme));
        if (url_host != client_ip) {
            send_ack(MSG_CMD_OTA_UPDATE, false, "url_not_allowed");
            return;
        }
    }
    send_ack(MSG_CMD_OTA_UPDATE, true);

    // Enable rainbow LEDs
    s_update_led_active = true;

    // Detach servos immediately — loop() can't drive ramps while this handler blocks.
    servos_detach_all();

    auto ota_cleanup = []() {
        s_update_led_active = false;
        sensor_led_set(1, 0, LED_BRIGHTNESS, 0);
        sensor_led_set(2, 0, LED_BRIGHTNESS, 0);
    };

    auto send_status = [](const char* status, const char* error = nullptr) {
        JsonDocument s;
        s["type"] = MSG_OTA_STATUS;
        s["status"] = status;
        if (error) s["error"] = error;
        send_json(s);
    };

    send_status("downloading");

    HTTPClient http;
    http.begin(url);
    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        send_status("failed", "http_error");
        http.end();
        ota_cleanup();
        return;
    }

    int len = http.getSize();
    WiFiClient* stream = http.getStreamPtr();
    if (!Update.begin(len > 0 ? len : UPDATE_SIZE_UNKNOWN)) {
        send_status("failed", "update_begin_failed");
        http.end();
        ota_cleanup();
        return;
    }

    send_status("flashing");

    // Chunk loop: feed Update and SHA-256 simultaneously
    uint8_t buf[4096];
    mbedtls_sha256_context sha_ctx;
    mbedtls_sha256_init(&sha_ctx);
    mbedtls_sha256_starts_ret(&sha_ctx, 0);  // 0 = SHA-256 (not SHA-224)

    bool write_error = false;
    bool timeout_error = false;
    size_t total_received = 0;
    unsigned long last_data = millis();
    while (http.connected()) {
        int avail = stream->available();
        if (avail <= 0) {
            // No data yet — check idle timeout then yield briefly and retry
            if (millis() - last_data > 10000UL) {
                timeout_error = true;
                break;
            }
            delay(1);
            continue;
        }
        int n = stream->readBytes(buf, sizeof(buf));
        if (n <= 0) {
            delay(1);
            continue;
        }
        last_data = millis();
        total_received += n;
        size_t written = Update.write(buf, n);
        if (written != (size_t)n) {
            write_error = true;
            break;
        }
        mbedtls_sha256_update_ret(&sha_ctx, buf, n);
    }

    uint8_t hash[32];
    int mret = mbedtls_sha256_finish_ret(&sha_ctx, hash);
    mbedtls_sha256_free(&sha_ctx);

    http.end();

    if (write_error) {
        Update.abort();
        send_status("failed", "flash_error");
        ota_cleanup();
        return;
    }

    if (timeout_error) {
        Update.abort();
        send_status("failed", "download_timeout");
        ota_cleanup();
        return;
    }

    if (len > 0 && total_received != (size_t)len) {
        Update.abort();
        send_status("failed", "incomplete_download");
        ota_cleanup();
        return;
    }

    if (mret != 0) {
        Update.abort();
        send_status("failed", "hash_compute_error");
        ota_cleanup();
        return;
    }

    // Optional SHA-256 verification — skip if field was absent
    if (expected_sha256 && expected_sha256[0] != '\0') {
        char actual_hex[65];
        for (int i = 0; i < 32; i++) {
            snprintf(actual_hex + i * 2, 3, "%02x", hash[i]);
        }
        actual_hex[64] = '\0';
        if (strncmp(actual_hex, expected_sha256, 64) != 0) {
            Update.abort();
            send_status("failed", "hash_mismatch");
            ota_cleanup();
            return;
        }
    }

    if (Update.end() && Update.isFinished()) {
        send_status("complete");
        s_update_led_active = false;
        delay(1000);
        ESP.restart();
    } else {
        send_status("failed", "flash_error");
        ota_cleanup();
    }
#endif
}

// --- Pin probe (servo identification tool) ---
// Uses LEDC channel 8 (not used by servos 0-7) to wiggle an arbitrary GPIO.
// Lets the user identify which physical servo is on which pin.
#define PROBE_LEDC_CHANNEL   8
#define PROBE_FREQ_HZ        50
#define PROBE_RESOLUTION     14
#define PROBE_AMPLITUDE_US   30
#define PROBE_HALF_PERIOD_MS 250
#define PROBE_CYCLES         4

static uint32_t probe_us_to_duty(uint16_t pulse_us) {
    // period = 20000us (50Hz); 14-bit = 16383 ticks max
    return (uint32_t)((uint64_t)pulse_us * 16383UL / 20000UL);
}

static void handle_cmd_probe_pin(const JsonDocument& doc) {
    uint8_t  pin    = doc["pin"]      | 255;
    uint16_t center = doc["pulse_us"] | 1500;

    // Reject sentinel / missing pin
    if (pin == 255) {
        send_ack(MSG_CMD_PROBE_PIN, false, "invalid_pin");
        return;
    }
    // Reject pins owned by firmware subsystems
    const char* reserved_reason = nullptr;
    if (pin_is_reserved(pin, &reserved_reason)) {
        char err_buf[32];
        snprintf(err_buf, sizeof(err_buf), "pin_reserved:%s",
                 reserved_reason ? reserved_reason : "unknown");
        send_ack(MSG_CMD_PROBE_PIN, false, err_buf);
        return;
    }
    // Reject if this pin is currently driving a live servo
    if (servos_engaged()) {
        for (int i = 0; i < 8; i++) {
            if (SERVO_PINS[i] == pin) {
                send_ack(MSG_CMD_PROBE_PIN, false, "servo_pin_engaged");
                return;
            }
        }
#if AUX_SERVOS_ENABLED
        for (int i = 0; i < AUX_SERVO_COUNT; i++) {
            if (AUX_SERVO_PINS[i] == pin) {
                send_ack(MSG_CMD_PROBE_PIN, false, "servo_pin_engaged");
                return;
            }
        }
#endif
    }
    if (center < 500)  center = 500;
    if (center > 2500) center = 2500;

    ledcSetup(PROBE_LEDC_CHANNEL, PROBE_FREQ_HZ, PROBE_RESOLUTION);
    ledcAttachPin(pin, PROBE_LEDC_CHANNEL);

    // Wiggle ±PROBE_AMPLITUDE_US around center, PROBE_CYCLES times
    for (int i = 0; i < PROBE_CYCLES; i++) {
        uint16_t hi = center + PROBE_AMPLITUDE_US;
        uint16_t lo = (center > PROBE_AMPLITUDE_US) ? (center - PROBE_AMPLITUDE_US) : 500u;
        ledcWrite(PROBE_LEDC_CHANNEL, probe_us_to_duty(hi));
        delay(PROBE_HALF_PERIOD_MS);
        ledcWrite(PROBE_LEDC_CHANNEL, probe_us_to_duty(lo));
        delay(PROBE_HALF_PERIOD_MS);
    }
    // Return to center, then detach
    ledcWrite(PROBE_LEDC_CHANNEL, probe_us_to_duty(center));
    delay(50);
    ledcDetachPin(pin);

    JsonDocument resp;
    resp["type"]     = MSG_ACK;
    resp["ref_type"] = MSG_CMD_PROBE_PIN;
    resp["ok"]       = true;
    resp["pin"]      = pin;
    send_json(resp);
}

static void handle_cmd_servo_pin(const JsonDocument& doc) {
    uint8_t idx = doc["index"] | 255;
    uint8_t pin = doc["pin"]   | 255;
    if (idx == 255 || pin == 255) {
        send_ack(MSG_CMD_SERVO_PIN, false, "missing_params");
        return;
    }
    String err;
    if (!servos_set_pin(idx, pin, err)) {
        send_ack(MSG_CMD_SERVO_PIN, false, err.c_str());
        return;
    }
    send_ack(MSG_CMD_SERVO_PIN, true);
    broadcast_servo_pins();
}

static void handle_cmd_buzzer(const JsonDocument& doc) {
    uint16_t freq = doc["freq_hz"]     | 2400;
    uint32_t dur  = doc["duration_ms"] | 200;
    buzzer_tone(freq, dur);
    send_ack(MSG_CMD_BUZZER, true);
}

static void handle_cmd_balance_config(const JsonDocument& doc) {
    float pitch_kp, pitch_ki, pitch_kd, roll_kp, roll_ki, roll_kd;
    balance_get_gains(&pitch_kp, &pitch_ki, &pitch_kd, &roll_kp, &roll_ki, &roll_kd);

    pitch_kp = doc["pitch_kp"] | pitch_kp;
    pitch_ki = doc["pitch_ki"] | pitch_ki;
    pitch_kd = doc["pitch_kd"] | pitch_kd;
    roll_kp  = doc["roll_kp"]  | roll_kp;
    roll_ki  = doc["roll_ki"]  | roll_ki;
    roll_kd  = doc["roll_kd"]  | roll_kd;

    balance_set_gains(pitch_kp, pitch_ki, pitch_kd, roll_kp, roll_ki, roll_kd);

    JsonDocument resp;
    resp["type"]     = MSG_ACK;
    resp["ref_type"] = MSG_CMD_BALANCE_CONFIG;
    resp["ok"]       = true;
    resp["pitch_kp"] = pitch_kp;
    resp["pitch_ki"] = pitch_ki;
    resp["pitch_kd"] = pitch_kd;
    resp["roll_kp"]  = roll_kp;
    resp["roll_ki"]  = roll_ki;
    resp["roll_kd"]  = roll_kd;
    send_json(resp);
}

static void handle_cmd_gpio(const JsonDocument& doc) {
    const char* op_str = doc["op"] | "";
    if (op_str[0] == '\0') {
        send_ack(MSG_CMD_GPIO, false, "missing op");
        return;
    }
    uint8_t pin = doc["pin"] | 255;
    if (!gpio_aux_allowlisted(pin)) {
        send_ack(MSG_CMD_GPIO, false, "pin_not_allowed");
        return;
    }

    if (strcmp(op_str, "mode") == 0) {
        const char* mode_str = doc["mode"] | "input_floating";
        GpioMode mode = GpioMode::FLOATING;
        if      (strcmp(mode_str, "input_pullup")   == 0) mode = GpioMode::GPIO_PULLUP;
        else if (strcmp(mode_str, "input_pulldown") == 0) mode = GpioMode::GPIO_PULLDOWN;
        else if (strcmp(mode_str, "output")         == 0) mode = GpioMode::GPIO_OUTPUT;
        gpio_aux_set_mode(pin, mode);
        send_ack(MSG_CMD_GPIO, true);

    } else if (strcmp(op_str, "write") == 0) {
        uint8_t val = doc["value"] | 0;
        gpio_aux_write(pin, val);
        send_ack(MSG_CMD_GPIO, true);

    } else if (strcmp(op_str, "read") == 0) {
        int digital_val = gpio_aux_read_digital(pin);
        int analog_val  = gpio_aux_read_analog(pin);
        JsonDocument resp;
        resp["type"]    = MSG_TELEM_GPIO;
        resp["pin"]     = pin;
        resp["digital"] = digital_val;
        resp["analog"]  = analog_val;
        send_json(resp);

    } else if (strcmp(op_str, "analog") == 0) {
        int analog_val  = gpio_aux_read_analog(pin);
        int digital_val = gpio_aux_read_digital(pin);
        JsonDocument resp;
        resp["type"]    = MSG_TELEM_GPIO;
        resp["pin"]     = pin;
        resp["digital"] = digital_val;
        resp["analog"]  = analog_val;
        send_json(resp);

    } else if (strcmp(op_str, "subscribe") == 0) {
        const char* mode_str = doc["mode"] | "input_floating";
        GpioMode mode = GpioMode::FLOATING;
        if      (strcmp(mode_str, "input_pullup")   == 0) mode = GpioMode::GPIO_PULLUP;
        else if (strcmp(mode_str, "input_pulldown") == 0) mode = GpioMode::GPIO_PULLDOWN;
        else if (strcmp(mode_str, "output")         == 0) mode = GpioMode::GPIO_OUTPUT;
        if (!sensor_gpio_subscribe(pin, mode)) {
            send_ack(MSG_CMD_GPIO, false, "subscribe_failed");
            return;
        }
        send_ack(MSG_CMD_GPIO, true);

    } else if (strcmp(op_str, "unsubscribe") == 0) {
        sensor_gpio_unsubscribe(pin);
        send_ack(MSG_CMD_GPIO, true);

    } else {
        send_ack(MSG_CMD_GPIO, false, "unknown_op");
    }
}

// --- Dispatch table ---

typedef void (*HandlerFn)(const JsonDocument&);
struct Handler { const char* type; HandlerFn fn; };

static const Handler k_handlers[] = {
    { MSG_PING,             handle_ping             },
    { MSG_CMD_ENGAGE,       handle_cmd_engage       },
    { MSG_CMD_MOVE,         handle_cmd_move         },
    { MSG_CMD_STAND,        handle_cmd_stand        },
    { MSG_CMD_BALANCE,      handle_cmd_balance      },
    { MSG_CMD_SERVO,        handle_cmd_servo        },
    { MSG_CMD_LED,          handle_cmd_led          },
    { MSG_CMD_TRANSFORM,    handle_cmd_transform    },
    { MSG_CMD_GAIT_PARAMS,  handle_cmd_gait_params  },
    { MSG_CMD_OFFSET,       handle_cmd_offset       },
    { MSG_CMD_I2C,          handle_cmd_i2c          },
    { MSG_CMD_OTA_UPDATE,     handle_cmd_ota_update     },
    { MSG_CMD_PROBE_PIN,      handle_cmd_probe_pin      },
    { MSG_CMD_BALANCE_CONFIG, handle_cmd_balance_config },
    { MSG_CMD_BUZZER,         handle_cmd_buzzer         },
    { MSG_CMD_GPIO,           handle_cmd_gpio           },
    { MSG_CMD_SERVO_PIN,      handle_cmd_servo_pin      },
};

void handlers_init() {
    // All state is owned by servos.cpp / gait.cpp / balance.cpp now — nothing to init here.
}

void handle_message(const JsonDocument& doc) {
    const char* type = doc["type"];
    if (!type) return;
    for (const auto& h : k_handlers) {
        if (strcmp(type, h.type) == 0) {
            h.fn(doc);
            return;
        }
    }
    send_ack(type, false, "unknown_type");
}
