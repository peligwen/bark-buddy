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

static GaitState dir_to_gait_state(const char* dir_str) {
    Direction dir = direction_from_string(dir_str);
    switch (dir) {
        case Direction::FORWARD:  return GaitState::WALK_FORWARD;
        case Direction::BACKWARD: return GaitState::WALK_BACKWARD;
        case Direction::LEFT:     return GaitState::TURN_LEFT;
        case Direction::RIGHT:    return GaitState::TURN_RIGHT;
        default:                  return GaitState::STOP;
    }
}


// --- Wake + queue helper ---
// Returns true if the lifecycle was not ACTIVE (caller should return).
// Stores cmd FIRST, then triggers wake — correct ordering required because
// lifecycle_cmd_wake() calls lifecycle_execute_pending() immediately when IDLE.
static bool wake_and_queue(const PendingCmd& cmd, const char* ack_type) {
    if (lifecycle_can_command()) return false;
    lifecycle_set_pending(cmd);   // store BEFORE wake
    lifecycle_cmd_wake(millis());
    send_ack(ack_type, true, "waking");
    return true;
}

// --- Handlers ---

static void handle_ping(const JsonDocument&) {
    JsonDocument resp;
    resp["type"] = MSG_PONG;
    send_json(resp);
}

static void handle_cmd_move(const JsonDocument& doc) {
    const char* dir_str = doc["direction"] | "stop";
    float spd = doc["speed"] | 1.0f;

    {
        PendingCmd cmd;
        cmd.type = PendingCmdType::MOVE;
        cmd.gait_state = dir_to_gait_state(dir_str);
        cmd.speed = spd;
        if (wake_and_queue(cmd, MSG_CMD_MOVE)) return;
    }
    s_manual_servo_mode = false;
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
    {
        PendingCmd cmd;
        cmd.type = PendingCmdType::STAND;
        if (wake_and_queue(cmd, MSG_CMD_STAND)) return;
    }
    s_manual_servo_mode = false;
    gait_set_state(GaitState::STAND);
    send_ack(MSG_CMD_STAND, true);
}

static void handle_cmd_balance(const JsonDocument& doc) {
    bool enabled = doc["enabled"] | true;

    {
        PendingCmd cmd;
        cmd.type = PendingCmdType::BALANCE;
        cmd.balance_enabled = enabled;
        if (wake_and_queue(cmd, MSG_CMD_BALANCE)) return;
    }
    s_balance_enabled = enabled;
    balance_enable(s_balance_enabled);
    if (!s_balance_enabled) balance_reset();
    send_ack(MSG_CMD_BALANCE, true);
}

static void handle_cmd_transform(const JsonDocument& doc) {
    BodyPose pose;
    pose.dx    = doc["x"]     | 0.0f;
    pose.dy    = doc["y"]     | 0.0f;
    pose.dz    = doc["z"]     | 0.0f;
    pose.roll  = doc["roll"]  | 0.0f;
    pose.pitch = doc["pitch"] | 0.0f;
    pose.yaw   = doc["yaw"]   | 0.0f;
    uint16_t ms = doc["ms"]   | 100;

    {
        PendingCmd cmd;
        cmd.type = PendingCmdType::TRANSFORM;
        cmd.body_pose = pose;
        cmd.transform_ms = ms;
        if (wake_and_queue(cmd, MSG_CMD_TRANSFORM)) return;
    }
    gait_set_body_transform(pose, ms);
    send_ack(MSG_CMD_TRANSFORM, true);
}

static void handle_cmd_gait_params(const JsonDocument& doc) {
    GaitConfig cfg;
    cfg.stride_height_mm = doc["stride_height"] | GAIT_STRIDE_HEIGHT_MM;
    cfg.stride_length_mm = doc["stride_length"] | GAIT_STRIDE_LENGTH_MM;
    cfg.frequency_hz     = doc["frequency"]     | GAIT_FREQUENCY_HZ;

    {
        PendingCmd cmd;
        cmd.type = PendingCmdType::GAIT_PARAMS;
        cmd.gait_config = cfg;
        if (wake_and_queue(cmd, MSG_CMD_GAIT_PARAMS)) return;
    }
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
    if (!lifecycle_can_command()) {
        lifecycle_cmd_wake(millis());
        JsonDocument evt;
        evt["type"]      = MSG_TELEM_EVENT;
        evt["event"]     = "command_rejected";
        evt["t"]         = (uint32_t)millis();
        evt["cmd"]       = MSG_CMD_SERVO;
        evt["reason"]    = "not_active";
        evt["lifecycle"] = lifecycle_state_name();
        send_json(evt);
        JsonDocument resp;
        resp["type"]      = MSG_ACK;
        resp["ref_type"]  = MSG_CMD_SERVO;
        resp["ok"]        = false;
        resp["error"]     = "not_active";
        resp["lifecycle"] = lifecycle_state_name();
        send_json(resp);
        return;
    }

    s_manual_servo_mode = true;

    uint8_t  idx = doc["index"]    | 0;
    uint16_t us  = doc["pulse_us"] | 1500;
    servo_write_us(idx, us);

    JsonDocument resp;
    resp["type"]      = MSG_ACK;
    resp["ref_type"]  = MSG_CMD_SERVO;
    resp["ok"]        = true;
    resp["index"]     = idx;
    resp["actual_us"] = servo_read_us(idx);
    send_json(resp);
#else
    send_ack(MSG_CMD_SERVO, false, "pins_not_verified");
#endif
}

static void handle_cmd_test_mode(const JsonDocument& doc) {
    bool enable = doc["enable"] | true;

    if (enable) {
        if (!lifecycle_can_command()) {
            lifecycle_cmd_wake(millis());
            JsonDocument evt;
            evt["type"]      = MSG_TELEM_EVENT;
            evt["event"]     = "command_rejected";
            evt["t"]         = (uint32_t)millis();
            evt["cmd"]       = MSG_CMD_TEST_MODE;
            evt["reason"]    = "not_active";
            evt["lifecycle"] = lifecycle_state_name();
            send_json(evt);
            JsonDocument resp;
            resp["type"]      = MSG_ACK;
            resp["ref_type"]  = MSG_CMD_TEST_MODE;
            resp["ok"]        = false;
            resp["error"]     = "not_active";
            resp["lifecycle"] = lifecycle_state_name();
            send_json(resp);
            return;
        }

        s_test_mode         = true;
        s_last_test_cmd     = millis();
        s_manual_servo_mode = true;
        gait_set_state(GaitState::STOP);

        sensor_led_set(1, LED_BRIGHTNESS / 2, 0, LED_BRIGHTNESS);  // purple
        sensor_led_set(2, LED_BRIGHTNESS / 2, 0, LED_BRIGHTNESS);
    } else {
        s_test_mode         = false;
        s_manual_servo_mode = false;
        gait_set_state(GaitState::STAND);

        sensor_led_set(1, 0, LED_BRIGHTNESS, 0);  // green
        sensor_led_set(2, 0, LED_BRIGHTNESS, 0);
    }

    JsonDocument resp;
    resp["type"]          = MSG_ACK;
    resp["ref_type"]      = MSG_CMD_TEST_MODE;
    resp["ok"]            = true;
    resp["test_mode"]     = s_test_mode;
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
    s_update_led_active = true;

    // Detach servos immediately — the ramp can't execute while loop() is blocked
    // by this handler. Detached is the same end-state as end-of-ramp anyway.
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

    // Reject invalid / reserved pins
    if (pin == 255 || pin == 22 || pin == 23 || pin >= 34) {
        send_ack(MSG_CMD_PROBE_PIN, false, "invalid_pin");
        return;
    }
    if (center < 500)  center = 500;
    if (center > 2500) center = 2500;

    ledcSetup(PROBE_LEDC_CHANNEL, PROBE_FREQ_HZ, PROBE_RESOLUTION);
    ledcAttachPin(pin, PROBE_LEDC_CHANNEL);

    // Wiggle ±PROBE_AMPLITUDE_US around center, PROBE_CYCLES times
    for (int i = 0; i < PROBE_CYCLES; i++) {
        uint16_t hi = center + PROBE_AMPLITUDE_US;
        uint16_t lo = (center > PROBE_AMPLITUDE_US) ? (center - PROBE_AMPLITUDE_US) : 500u;
        ledcWrite(pin, probe_us_to_duty(hi));
        delay(PROBE_HALF_PERIOD_MS);
        ledcWrite(pin, probe_us_to_duty(lo));
        delay(PROBE_HALF_PERIOD_MS);
    }
    // Return to center, then detach
    ledcWrite(pin, probe_us_to_duty(center));
    delay(50);
    ledcDetachPin(pin);

    JsonDocument resp;
    resp["type"]     = MSG_ACK;
    resp["ref_type"] = MSG_CMD_PROBE_PIN;
    resp["ok"]       = true;
    resp["pin"]      = pin;
    send_json(resp);
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

// --- Update begin (serial flash prep) ---
// Triggered by the host before a serial upload. Starts rainbow LEDs and detaches
// servos so the dog is safe to receive a reset/flash.
static void handle_cmd_update_begin(const JsonDocument& doc) {
    servos_detach_all();
    s_update_led_active = true;
    send_ack(MSG_CMD_UPDATE_BEGIN, true);
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
    { MSG_CMD_OTA_UPDATE,     handle_cmd_ota_update     },
    { MSG_CMD_UPDATE_BEGIN,   handle_cmd_update_begin   },
    { MSG_CMD_PROBE_PIN,      handle_cmd_probe_pin      },
    { MSG_CMD_BALANCE_CONFIG, handle_cmd_balance_config },
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
        if (strcmp(type, h.type) == 0) {
            h.fn(doc);
            // Refresh test-mode heartbeat for any command received while in test mode.
            // Done after the handler so cmd_test_mode{enable:true} is already applied.
            if (s_test_mode) s_last_test_cmd = millis();
            return;
        }
    }
    send_ack(type, false, "unknown_type");
}

void handlers_check_timeout(unsigned long now_ms) {
    if (!s_test_mode) return;
    if (now_ms - s_last_test_cmd <= TEST_HEARTBEAT_MS) return;

    JsonDocument evt;
    evt["type"]              = MSG_TELEM_EVENT;
    evt["event"]             = "test_mode_timeout";
    evt["t"]                 = (uint32_t)now_ms;
    evt["ms_since_last_cmd"] = (uint32_t)(now_ms - s_last_test_cmd);
    send_json(evt);

    s_test_mode         = false;
    s_manual_servo_mode = false;
    gait_set_state(GaitState::STAND);

    sensor_led_set(1, 0, LED_BRIGHTNESS, 0);  // green
    sensor_led_set(2, 0, LED_BRIGHTNESS, 0);
}
