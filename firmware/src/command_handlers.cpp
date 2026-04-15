// firmware/src/command_handlers.cpp
#include "command_handlers.h"
#include "comms.h"
#include "sensor_task.h"
#include "protocol.h"
#include "config.h"
#include "gait.h"
#include "servos.h"
#include "balance.h"
#include "offsets.h"
#include "ota.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <string.h>
#if WIFI_ENABLED
#include <HTTPClient.h>
#include <Update.h>
#include <mbedtls/sha256.h>
#endif

// --- Handler-owned state ---
static bool          s_balance_enabled    = false;
static bool          s_manual_servo_mode  = false;
static bool          s_test_mode          = false;
static unsigned long s_last_test_cmd      = 0;

// --- Accessors for main loop ---
bool          handlers_manual_servo_mode() { return s_manual_servo_mode; }
bool          handlers_test_mode()         { return s_test_mode; }
unsigned long handlers_last_test_cmd()     { return s_last_test_cmd; }

// --- Direction helpers (moved from main.cpp) ---
Direction direction_from_string(const char* str) {
    if (strcmp(str, "forward")  == 0) return Direction::FORWARD;
    if (strcmp(str, "backward") == 0) return Direction::BACKWARD;
    if (strcmp(str, "left")     == 0) return Direction::LEFT;
    if (strcmp(str, "right")    == 0) return Direction::RIGHT;
    return Direction::STOP;
}


// --- Handlers ---

static void handle_ping(const JsonDocument&) {
    JsonDocument resp;
    resp["type"] = MSG_PONG;
    send_json(resp);
}

static void handle_cmd_move(const JsonDocument& doc) {
    if (!lifecycle_can_command()) {
        send_ack(MSG_CMD_MOVE, false, "not_active");
        return;
    }
    s_manual_servo_mode = false;
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
    if (!lifecycle_can_command()) {
        send_ack(MSG_CMD_STAND, false, "not_active");
        return;
    }
    s_manual_servo_mode = false;
    gait_set_state(GaitState::STAND);
    send_ack(MSG_CMD_STAND, true);
}

static void handle_cmd_balance(const JsonDocument& doc) {
    if (!lifecycle_can_command()) {
        send_ack(MSG_CMD_BALANCE, false, "not_active");
        return;
    }
    s_balance_enabled = doc["enabled"] | true;
    balance_enable(s_balance_enabled);
    if (!s_balance_enabled) balance_reset();
    send_ack(MSG_CMD_BALANCE, true);
}

static void handle_cmd_transform(const JsonDocument& doc) {
    if (!lifecycle_can_command()) {
        send_ack(MSG_CMD_TRANSFORM, false, "not_active");
        return;
    }
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
    if (!lifecycle_can_command()) {
        send_ack(MSG_CMD_GAIT_PARAMS, false, "not_active");
        return;
    }
    GaitConfig cfg;
    cfg.stride_height_mm = doc["stride_height"] | GAIT_STRIDE_HEIGHT_MM;
    cfg.stride_length_mm = doc["stride_length"] | GAIT_STRIDE_LENGTH_MM;
    cfg.frequency_hz     = doc["frequency"]     | GAIT_FREQUENCY_HZ;
    gait_set_config(cfg);
    send_ack(MSG_CMD_GAIT_PARAMS, true);
}

static void handle_cmd_led(const JsonDocument& doc) {
    uint8_t led = doc["led"] | 1;
    uint8_t r   = doc["r"]   | 0;
    uint8_t g   = doc["g"]   | 0;
    uint8_t b   = doc["b"]   | 0;
    sensor_led_set(led, r, g, b);
    send_ack(MSG_CMD_LED, true);
}

static void handle_cmd_servo(const JsonDocument& doc) {
#if PINS_VERIFIED
    s_manual_servo_mode = true;
    if (s_test_mode) s_last_test_cmd = millis();

    if (!servos_active()) {
        servos_init();
        if (s_test_mode) servos_set_frail(true);
    }

    uint8_t  idx = doc["index"]    | 0;
    uint16_t us  = doc["pulse_us"] | 1500;
    servo_write_us(idx, us);

    JsonDocument resp;
    resp["type"]      = MSG_ACK;
    resp["ref_type"]  = MSG_CMD_SERVO;
    resp["ok"]        = true;
    resp["index"]     = idx;
    resp["actual_us"] = servo_read_us(idx);
    resp["frail"]     = servos_frail();
    send_json(resp);
#else
    send_ack(MSG_CMD_SERVO, false, "pins_not_verified");
#endif
}

static void handle_cmd_test_mode(const JsonDocument& doc) {
    bool enable = doc["enable"] | true;
    bool frail  = doc["frail"]  | true;

    if (enable) {
        s_test_mode         = true;
        s_last_test_cmd     = millis();
        s_manual_servo_mode = true;
        gait_set_state(GaitState::STOP);

        if (!servos_active()) servos_init();
        servos_set_frail(frail);

        sensor_led_set(1, LED_BRIGHTNESS / 2, 0, LED_BRIGHTNESS);  // purple
        sensor_led_set(2, LED_BRIGHTNESS / 2, 0, LED_BRIGHTNESS);
    } else {
        s_test_mode         = false;
        s_manual_servo_mode = false;
        servos_set_frail(false);
        gait_set_state(GaitState::STAND);

        sensor_led_set(1, 0, LED_BRIGHTNESS, 0);  // green
        sensor_led_set(2, 0, LED_BRIGHTNESS, 0);
    }

    JsonDocument resp;
    resp["type"]          = MSG_ACK;
    resp["ref_type"]      = MSG_CMD_TEST_MODE;
    resp["ok"]            = true;
    resp["test_mode"]     = s_test_mode;
    resp["frail"]         = servos_frail();
    resp["servos_active"] = servos_active();
    send_json(resp);
}

static void handle_cmd_wake(const JsonDocument&) {
    lifecycle_cmd_wake(millis());
    send_ack(MSG_CMD_WAKE, true);
}

static void handle_cmd_sleep(const JsonDocument&) {
    lifecycle_cmd_sleep(millis());
    send_ack(MSG_CMD_SLEEP, true);
}

static void handle_cmd_shutdown(const JsonDocument&) {
    lifecycle_cmd_shutdown(millis());
    send_ack(MSG_CMD_SHUTDOWN, true);
}

static void handle_cmd_i2c_write(const JsonDocument& doc) {
    uint8_t addr = doc["addr"] | 0x77;
    uint8_t reg  = doc["reg"]  | 0;
    uint8_t val  = doc["val"]  | 0;
    bool ok = sensor_i2c_write(addr, reg, val);

    JsonDocument resp;
    resp["type"]     = MSG_ACK;
    resp["ref_type"] = MSG_CMD_I2C_WRITE;
    resp["ok"]       = ok;
    resp["addr"]     = addr;
    resp["reg"]      = reg;
    resp["val"]      = val;
    send_json(resp);
}

static void handle_cmd_offset(const JsonDocument& doc) {
    const char* action = doc["action"] | "read";
    if (strcmp(action, "set") == 0) {
        uint8_t idx = doc["index"] | 0;
        int16_t val = doc["value"] | 0;
        if (idx < 8) offset_set(idx, val);
    } else if (strcmp(action, "save") == 0) {
        offsets_save();
    } else if (strcmp(action, "reset") == 0) {
        offsets_reset();
    }
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
    // Must be IDLE or ACTIVE
    LifecycleState lc = lifecycle_current();
    if (lc != LifecycleState::IDLE && lc != LifecycleState::ACTIVE) {
        send_ack(MSG_CMD_OTA_UPDATE, false, "not_idle_or_active");
        return;
    }
    send_ack(MSG_CMD_OTA_UPDATE, true);

    // Transition to UPDATING (ramps to rest pose)
    lifecycle_cmd_update(millis());

    // Enable rainbow LEDs
    s_ota_led_active = true;

    // Detach servos immediately — the ramp can't execute while loop() is blocked
    // by this handler. Detached is the same end-state as end-of-ramp anyway.
    servos_detach_all();

    auto ota_cleanup = [&]() {
        s_ota_led_active = false;
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
    while (http.connected()) {
        int avail = stream->available();
        if (avail <= 0) {
            // No data yet — yield briefly and retry
            delay(1);
            continue;
        }
        int n = stream->readBytes(buf, sizeof(buf));
        if (n <= 0) {
            delay(1);
            continue;
        }
        size_t written = Update.write(buf, n);
        if (written != (size_t)n) {
            write_error = true;
            break;
        }
        mbedtls_sha256_update_ret(&sha_ctx, buf, n);
    }

    uint8_t hash[32];
    mbedtls_sha256_finish_ret(&sha_ctx, hash);
    mbedtls_sha256_free(&sha_ctx);

    http.end();

    if (write_error) {
        Update.abort();
        send_status("failed", "flash_error");
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
        s_ota_led_active = false;
        delay(1000);
        ESP.restart();
    } else {
        send_status("failed", "flash_error");
        ota_cleanup();
    }
#endif
}

// --- Dispatch table ---

typedef void (*HandlerFn)(const JsonDocument&);
struct Handler { const char* type; HandlerFn fn; };

static const Handler k_handlers[] = {
    { MSG_PING,             handle_ping             },
    { MSG_CMD_MOVE,         handle_cmd_move         },
    { MSG_CMD_STAND,        handle_cmd_stand        },
    { MSG_CMD_BALANCE,      handle_cmd_balance      },
    { MSG_CMD_SERVO,        handle_cmd_servo        },
    { MSG_CMD_LED,          handle_cmd_led          },
    { MSG_CMD_TRANSFORM,    handle_cmd_transform    },
    { MSG_CMD_GAIT_PARAMS,  handle_cmd_gait_params  },
    { MSG_CMD_TEST_MODE,    handle_cmd_test_mode    },
    { MSG_CMD_OFFSET,       handle_cmd_offset       },
    { MSG_CMD_I2C_WRITE,    handle_cmd_i2c_write    },
    { MSG_CMD_SHUTDOWN,     handle_cmd_shutdown     },
    { MSG_CMD_WAKE,         handle_cmd_wake         },
    { MSG_CMD_SLEEP,        handle_cmd_sleep        },
    { MSG_CMD_OTA_UPDATE,   handle_cmd_ota_update   },
};

void handlers_init() {
    s_balance_enabled   = false;
    s_manual_servo_mode = false;
    s_test_mode         = false;
    s_last_test_cmd     = 0;
}

void handle_message(const JsonDocument& doc) {
    const char* type = doc["type"];
    if (!type) return;
    for (const auto& h : k_handlers) {
        if (strcmp(type, h.type) == 0) { h.fn(doc); return; }
    }
    send_ack(type, false, "unknown_type");
}

void handlers_check_timeout(unsigned long now_ms) {
    if (!s_test_mode) return;
    if (now_ms - s_last_test_cmd <= TEST_HEARTBEAT_MS) return;

    s_test_mode         = false;
    s_manual_servo_mode = false;
    servos_set_frail(false);
    gait_set_state(GaitState::STAND);

    sensor_led_set(1, 0, LED_BRIGHTNESS, 0);  // green
    sensor_led_set(2, 0, LED_BRIGHTNESS, 0);
}
