// firmware/src/command_handlers.cpp
//
// Dispatch table + motion / IO / config command handlers.
//
// OTA-related handlers live in command_handlers_ota.cpp.
// Pin-probe lives in command_handlers_probe.cpp.
// All three translation units share `command_handlers_internal.h`.
//
// All handlers take `(const JsonDocument&, MsgSource)`. Most ignore the
// source argument; only OTA gates auth on it.

#include "command_handlers.h"
#include "command_handlers_internal.h"
#include "comms.h"
#include "protocol.h"
#include "sensor_task.h"
#include "config.h"
#include "gait.h"
#include "servos.h"
#include "balance.h"
#include "offsets.h"
#include "buzzer.h"
#include "gpio_aux.h"
#include "pin_registry.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <string.h>

// --- Helpers ---

// Motion commands require engaged + non-ramping + battery present.
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

// "input_floating" | "input_pullup" | "input_pulldown" | "output" → GpioMode.
// Unknown strings fall back to FLOATING.
static GpioMode parse_gpio_mode(const char* mode_str) {
    if (!mode_str) return GpioMode::FLOATING;
    if (strcmp(mode_str, "input_pullup")   == 0) return GpioMode::GPIO_PULLUP;
    if (strcmp(mode_str, "input_pulldown") == 0) return GpioMode::GPIO_PULLDOWN;
    if (strcmp(mode_str, "output")         == 0) return GpioMode::GPIO_OUTPUT;
    return GpioMode::FLOATING;
}

void broadcast_servo_pins() {
    JsonDocument resp;
    resp["type"] = MSG_TELEM_SERVO_PINS;
    JsonArray arr = resp["pins"].to<JsonArray>();
    for (int i = 0; i < 8; i++) arr.add(SERVO_PINS[i]);
    send_json(resp);
}

// --- Handlers ---

static void handle_ping(const JsonDocument&, MsgSource) {
    JsonDocument resp;
    resp["type"] = MSG_PONG;
    char core_ver[16];
    snprintf(core_ver, sizeof(core_ver), "%d.%d.%d",
        ESP_ARDUINO_VERSION_MAJOR,
        ESP_ARDUINO_VERSION_MINOR,
        ESP_ARDUINO_VERSION_PATCH);
    resp["arduino_esp32_core"] = core_ver;
#ifdef PIO_PLATFORM_VER
    resp["pio_platform"] = PIO_PLATFORM_VER;
#endif
    send_json(resp);
}

static void handle_cmd_engage(const JsonDocument& doc, MsgSource) {
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
            send_ack(MSG_CMD_ENGAGE, true);
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
            send_ack(MSG_CMD_ENGAGE, true);
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

static void handle_cmd_move(const JsonDocument& doc, MsgSource) {
    if (!require_engaged(MSG_CMD_MOVE)) return;
    const char* dir_str = doc["direction"] | "stop";
    float spd = doc["speed"] | 1.0f;
    GaitState target;
    if      (strcmp(dir_str, "forward")      == 0) target = GaitState::WALK_FORWARD;
    else if (strcmp(dir_str, "backward")     == 0) target = GaitState::WALK_BACKWARD;
    else if (strcmp(dir_str, "left")         == 0 ||
             strcmp(dir_str, "rotate_left")  == 0) target = GaitState::TURN_LEFT;
    else if (strcmp(dir_str, "right")        == 0 ||
             strcmp(dir_str, "rotate_right") == 0) target = GaitState::TURN_RIGHT;
    else                                            target = GaitState::STOP;
    gait_set_state(target, spd);
    send_ack(MSG_CMD_MOVE, true);
}

static void handle_cmd_stand(const JsonDocument&, MsgSource) {
    if (!require_engaged(MSG_CMD_STAND)) return;
    gait_set_state(GaitState::STAND);
    send_ack(MSG_CMD_STAND, true);
}

static void handle_cmd_balance(const JsonDocument& doc, MsgSource) {
    if (!require_engaged(MSG_CMD_BALANCE)) return;
    bool enabled = doc["enabled"] | true;
    balance_enable(enabled);
    if (!enabled) balance_reset();
    send_ack(MSG_CMD_BALANCE, true);
}

static void handle_cmd_transform(const JsonDocument& doc, MsgSource) {
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

static void handle_cmd_gait_params(const JsonDocument& doc, MsgSource) {
    if (!require_engaged(MSG_CMD_GAIT_PARAMS)) return;
    GaitConfig cfg;
    cfg.stride_height_mm = doc["stride_height"]  | GAIT_STRIDE_HEIGHT_MM;
    cfg.stride_length_mm = doc["stride_length"]  | GAIT_STRIDE_LENGTH_MM;
    cfg.frequency_hz     = doc["frequency"]      | GAIT_FREQUENCY_HZ;
    cfg.swing_time_ms    = doc["swing_time_ms"]  | (uint32_t)GAIT_SWING_TIME_MS;
    cfg.stand_time_ms    = doc["stand_time_ms"]  | (uint32_t)GAIT_STAND_TIME_MS;
    gait_set_config(cfg);
    send_ack(MSG_CMD_GAIT_PARAMS, true);
}

static void handle_cmd_led(const JsonDocument& doc, MsgSource) {
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

static void handle_cmd_servo(const JsonDocument& doc, MsgSource) {
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

static void handle_cmd_i2c(const JsonDocument& doc, MsgSource) {
    const char* op_str = doc["op"] | "";
    if (op_str[0] == '\0') {
        send_ack(MSG_CMD_I2C, false, "missing op");
        return;
    }
    uint8_t bus = doc["bus"] | 1;

    I2cOp op;
    if      (strcmp(op_str, "scan")  == 0) op = I2cOp::SCAN;
    else if (strcmp(op_str, "read")  == 0) op = I2cOp::READ;
    else if (strcmp(op_str, "write") == 0) op = I2cOp::WRITE;
    else {
        send_ack(MSG_CMD_I2C, false, "unknown_op");
        return;
    }

    // SCAN stalls the sensor task ~130 ms — refuse during active motion.
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

static void handle_cmd_offset(const JsonDocument& doc, MsgSource) {
    // No engagement required — offsets are NVS I/O only.
    // op="set"  (default): update RAM only — safe for rapid slider scrubbing.
    // op="save": flush RAM to NVS. Keeping set/save separate avoids wearing
    // flash during calibration.
    const char* op = doc["op"] | "set";
    if (!doc["offset_us"].isNull()) {
        uint8_t idx = doc["index"] | 255;
        if (idx >= 8) {
            send_ack(MSG_CMD_OFFSET, false, "bad_index");
            return;
        }
        offset_set(idx, doc["offset_us"] | 0);
    }
    if (strcmp(op, "save") == 0) offsets_save();

    JsonDocument resp;
    resp["type"]     = MSG_ACK;
    resp["ref_type"] = MSG_CMD_OFFSET;
    resp["ok"]       = true;
    JsonArray arr = resp["offsets"].to<JsonArray>();
    for (int i = 0; i < 8; i++) arr.add(offset_get(i));
    send_json(resp);
}

static void handle_cmd_servo_pin(const JsonDocument& doc, MsgSource) {
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

static void handle_cmd_yaw_trim(const JsonDocument& doc, MsgSource) {
    const char* op = doc["op"] | "get";
    if (strcmp(op, "set") == 0) {
        gait_set_yaw_trim(doc["value"] | 0.0f);
    } else if (strcmp(op, "save") == 0) {
        gait_save_yaw_trim();
    }
    JsonDocument resp;
    resp["type"]     = MSG_ACK;
    resp["ref_type"] = MSG_CMD_YAW_TRIM;
    resp["ok"]       = true;
    resp["value"]    = gait_get_yaw_trim();
    send_json(resp);
}

static void handle_cmd_buzzer(const JsonDocument& doc, MsgSource) {
    uint16_t freq = doc["freq_hz"]     | 2400;
    uint32_t dur  = doc["duration_ms"] | 200;
    buzzer_tone(freq, dur);
    send_ack(MSG_CMD_BUZZER, true);
}

static void handle_cmd_balance_config(const JsonDocument& doc, MsgSource) {
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

static void handle_cmd_gpio(const JsonDocument& doc, MsgSource) {
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
        gpio_aux_set_mode(pin, parse_gpio_mode(doc["mode"] | "input_floating"));
        send_ack(MSG_CMD_GPIO, true);

    } else if (strcmp(op_str, "write") == 0) {
        gpio_aux_write(pin, doc["value"] | 0);
        send_ack(MSG_CMD_GPIO, true);

    } else if (strcmp(op_str, "read") == 0 || strcmp(op_str, "analog") == 0) {
        // read/analog are aliases — both return digital + analog in telem_gpio.
        JsonDocument resp;
        resp["type"]    = MSG_TELEM_GPIO;
        resp["pin"]     = pin;
        resp["digital"] = gpio_aux_read_digital(pin);
        resp["analog"]  = gpio_aux_read_analog(pin);
        send_json(resp);

    } else if (strcmp(op_str, "subscribe") == 0) {
        if (!sensor_gpio_subscribe(pin, parse_gpio_mode(doc["mode"] | "input_floating"))) {
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

typedef void (*HandlerFn)(const JsonDocument&, MsgSource);
struct Handler { const char* type; HandlerFn fn; };

static const Handler k_handlers[] = {
    { MSG_PING,                  handle_ping                  },
    { MSG_CMD_ENGAGE,            handle_cmd_engage            },
    { MSG_CMD_MOVE,              handle_cmd_move              },
    { MSG_CMD_STAND,             handle_cmd_stand             },
    { MSG_CMD_BALANCE,           handle_cmd_balance           },
    { MSG_CMD_SERVO,             handle_cmd_servo             },
    { MSG_CMD_LED,               handle_cmd_led               },
    { MSG_CMD_TRANSFORM,         handle_cmd_transform         },
    { MSG_CMD_GAIT_PARAMS,       handle_cmd_gait_params       },
    { MSG_CMD_OFFSET,            handle_cmd_offset            },
    { MSG_CMD_I2C,               handle_cmd_i2c               },
    { MSG_CMD_OTA_UPDATE,        handle_cmd_ota_update        },  // ota.cpp
    { MSG_CMD_OTA_REQUEST_NONCE, handle_cmd_ota_request_nonce },  // ota.cpp
    { MSG_CMD_PROBE_PIN,         [](const JsonDocument& d, MsgSource){ handle_cmd_probe_pin(d); } },
    { MSG_CMD_BALANCE_CONFIG,    handle_cmd_balance_config    },
    { MSG_CMD_BUZZER,            handle_cmd_buzzer            },
    { MSG_CMD_GPIO,              handle_cmd_gpio              },
    { MSG_CMD_SERVO_PIN,         handle_cmd_servo_pin         },
    { MSG_CMD_YAW_TRIM,          handle_cmd_yaw_trim          },
};

void handlers_init() {
    // No handler-owned state — kept as a hook for future setup work.
}

void handle_message(const JsonDocument& doc, MsgSource source) {
    const char* type = doc["type"];
    if (!type) return;
    for (const auto& h : k_handlers) {
        if (strcmp(type, h.type) == 0) {
            h.fn(doc, source);
            return;
        }
    }
    send_ack(type, false, "unknown_type");
}
