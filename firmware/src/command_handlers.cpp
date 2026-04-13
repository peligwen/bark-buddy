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
#include <Arduino.h>
#include <ArduinoJson.h>
#include <string.h>

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

// --- Handlers ---

static void handle_ping(const JsonDocument&) {
    JsonDocument resp;
    resp["type"] = MSG_PONG;
    send_json(resp);
}

static void handle_cmd_move(const JsonDocument& doc) {
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
    s_manual_servo_mode = false;
    gait_set_state(GaitState::STAND);
    send_ack(MSG_CMD_STAND, true);
}

static void handle_cmd_balance(const JsonDocument& doc) {
    s_balance_enabled = doc["enabled"] | true;
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
    gait_set_body_transform(pose, ms);
    send_ack(MSG_CMD_TRANSFORM, true);
}

static void handle_cmd_gait_params(const JsonDocument& doc) {
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
