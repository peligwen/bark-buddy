# Firmware Foundation Refactor — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the custom ESP32 firmware into three focused units — a FreeRTOS sensor task that owns the I2C bus, a handler dispatch table for commands, and non-blocking WiFi reconnect — without changing any protocol behavior.

**Architecture:** A FreeRTOS sensor task owns all I2C access (IMU, sonar, LEDs, debug writes) and publishes results to a mutex-protected `SensorSnapshot` struct. The main Arduino loop reads from the snapshot and dispatches commands via a flat handler table. WiFi connects non-blocking in `setup()` and auto-reconnects in `loop()`.

**Tech Stack:** C++17, PlatformIO, ESP32 Arduino framework, FreeRTOS (built-in to ESP32 Arduino), ArduinoJson

**Spec:** `docs/superpowers/specs/2026-04-13-firmware-foundation-design.md`

---

## File Map

| Action | Path | Responsibility |
|---|---|---|
| Create | `firmware/include/comms.h` | Forward-declares `send_json` / `send_ack` for cross-file use |
| Create | `firmware/include/sensor_task.h` | Public API: `SensorSnapshot`, queue structs, 4 functions |
| Create | `firmware/src/sensor_task.cpp` | FreeRTOS task, Wire init, snapshot mutex, LED + I2C queues |
| Create | `firmware/include/command_handlers.h` | Public API: `handlers_init`, `handle_message`, 3 state accessors |
| Create | `firmware/src/command_handlers.cpp` | Handler table + all 11 `handle_*` functions, direction helpers |
| Modify | `firmware/src/main.cpp` | Use sensor_task + handlers; non-blocking WiFi; ~150 lines |

All other firmware files are untouched.

---

## Task 1: Add `comms.h` — shared send declarations

`send_json` and `send_ack` are defined in `main.cpp` but called by `command_handlers.cpp`. They need a shared declaration.

**Files:**
- Create: `firmware/include/comms.h`

- [ ] **Step 1: Create the header**

```cpp
// firmware/include/comms.h
#pragma once
#include <ArduinoJson.h>

// Defined in main.cpp. Writes NDJSON to serial and (if connected) TCP client.
void send_json(const JsonDocument& doc);

// Convenience wrapper: sends {"type":"ack","ref_type":ref_type,"ok":ok[,"error":error]}
void send_ack(const char* ref_type, bool ok, const char* error = nullptr);
```

- [ ] **Step 2: Verify existing tests still build**

```bash
make -C firmware/test
```

Expected: all targets build cleanly, no errors.

- [ ] **Step 3: Commit**

```bash
git add firmware/include/comms.h
git commit -m "feat(firmware): add comms.h shared send declarations"
```

---

## Task 2: Create `sensor_task.h`

Define the public interface for the sensor task before writing the implementation.

**Files:**
- Create: `firmware/include/sensor_task.h`

- [ ] **Step 1: Create the header**

```cpp
// firmware/include/sensor_task.h
#pragma once
#include <stdint.h>

// Latest values from IMU and sonar. Written by sensor task, read by main loop.
struct SensorSnapshot {
    float    pitch, roll, yaw;   // degrees (complementary filter)
    float    ax, ay, az;          // m/s²
    float    gx, gy, gz;          // deg/s
    uint16_t sonar_mm;
    bool     imu_ok;
    bool     sonar_ok;
};

struct LedCmd      { uint8_t led, r, g, b; };
struct I2cWriteCmd { uint8_t addr, reg, val; };

// Start the FreeRTOS sensor task. Initialises I2C, IMU, and sonar inside the
// task. Blocks until the first init pass completes (≤1s) so that imu_ok and
// sonar_ok in the snapshot are valid before setup() sends the boot message.
void sensor_task_start();

// Copy the latest sensor values into `out`. Thread-safe; takes snapshot mutex.
void sensor_snapshot_get(SensorSnapshot& out);

// Queue a LED colour change. Non-blocking — drops silently if the queue is
// full (depth 4). Safe to call from main loop or command handlers.
void sensor_led_set(uint8_t led, uint8_t r, uint8_t g, uint8_t b);

// Issue a raw I2C register write through the sensor task. Blocks until the
// task executes it (≤~10ms). Debug / probe only. Returns true if ACKed.
bool sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t val);
```

- [ ] **Step 2: Verify tests still build**

```bash
make -C firmware/test
```

Expected: all targets build, no new errors.

- [ ] **Step 3: Commit**

```bash
git add firmware/include/sensor_task.h
git commit -m "feat(firmware): add sensor_task.h interface"
```

---

## Task 3: Implement `sensor_task.cpp`

The sensor task owns the I2C bus from `Wire.begin()` onward. Nothing in `main.cpp` or the handlers ever calls `Wire` directly after this task exists.

**Files:**
- Create: `firmware/src/sensor_task.cpp`

- [ ] **Step 1: Create the implementation**

```cpp
// firmware/src/sensor_task.cpp
#include "sensor_task.h"
#include "imu.h"
#include "sonar.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

static SensorSnapshot    s_snapshot         = {};
static SemaphoreHandle_t s_snapshot_mutex   = nullptr;
static SemaphoreHandle_t s_ready_sem        = nullptr;
static QueueHandle_t     s_led_queue        = nullptr;
static QueueHandle_t     s_i2c_write_queue  = nullptr;
static SemaphoreHandle_t s_i2c_write_done   = nullptr;
static bool              s_i2c_write_result = false;

static void sensor_task_fn(void*) {
    // Take ownership of the I2C bus
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);

    bool imu_ok   = imu_init(Wire);
    bool sonar_ok = sonar_init(Wire);

    // Set boot LED based on init results
    if (imu_ok && sonar_ok) {
        sonar_set_rgb(1, LED_R_LAVENDER, LED_G_LAVENDER, LED_B_LAVENDER);
        sonar_set_rgb(2, LED_R_LAVENDER, LED_G_LAVENDER, LED_B_LAVENDER);
    } else {
        sonar_set_rgb(1, LED_BRIGHTNESS, 0, 0);
        sonar_set_rgb(2, LED_BRIGHTNESS, 0, 0);
    }

    // Publish init results and unblock sensor_task_start()
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    s_snapshot.imu_ok   = imu_ok;
    s_snapshot.sonar_ok = sonar_ok;
    xSemaphoreGive(s_snapshot_mutex);
    xSemaphoreGive(s_ready_sem);

    unsigned long last_imu   = 0;
    unsigned long last_sonar = 0;

    for (;;) {
        unsigned long now = millis();

        // Read IMU at TELEM_IMU_HZ
        if (now - last_imu >= 1000 / TELEM_IMU_HZ) {
            IMUData d;
            if (imu_read(d)) {
                xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
                s_snapshot.pitch = d.pitch;
                s_snapshot.roll  = d.roll;
                s_snapshot.yaw   = d.yaw;
                s_snapshot.ax = d.ax; s_snapshot.ay = d.ay; s_snapshot.az = d.az;
                s_snapshot.gx = d.gx; s_snapshot.gy = d.gy; s_snapshot.gz = d.gz;
                xSemaphoreGive(s_snapshot_mutex);
            }
            last_imu = now;
        }

        // Read sonar at TELEM_SONAR_HZ
        if (now - last_sonar >= 1000 / TELEM_SONAR_HZ) {
            uint16_t dist = sonar_read_mm();
            xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
            s_snapshot.sonar_mm = dist;
            xSemaphoreGive(s_snapshot_mutex);
            last_sonar = now;
        }

        // Drain LED queue — process all pending colour changes
        LedCmd led;
        while (xQueueReceive(s_led_queue, &led, 0)) {
            sonar_set_rgb(led.led, led.r, led.g, led.b);
        }

        // Service a pending raw I2C write (debug only)
        I2cWriteCmd i2c;
        if (xQueueReceive(s_i2c_write_queue, &i2c, 0)) {
            Wire.beginTransmission(i2c.addr);
            Wire.write(i2c.reg);
            Wire.write(i2c.val);
            s_i2c_write_result = (Wire.endTransmission() == 0);
            xSemaphoreGive(s_i2c_write_done);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void sensor_task_start() {
    s_snapshot_mutex  = xSemaphoreCreateMutex();
    s_ready_sem       = xSemaphoreCreateBinary();
    s_led_queue       = xQueueCreate(4, sizeof(LedCmd));
    s_i2c_write_queue = xQueueCreate(1, sizeof(I2cWriteCmd));
    s_i2c_write_done  = xSemaphoreCreateBinary();

    xTaskCreate(sensor_task_fn, "sensor", 4096, nullptr, 2, nullptr);

    // Block until task completes its init pass
    xSemaphoreTake(s_ready_sem, pdMS_TO_TICKS(1000));
}

void sensor_snapshot_get(SensorSnapshot& out) {
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    out = s_snapshot;
    xSemaphoreGive(s_snapshot_mutex);
}

void sensor_led_set(uint8_t led, uint8_t r, uint8_t g, uint8_t b) {
    LedCmd cmd = {led, r, g, b};
    xQueueSend(s_led_queue, &cmd, 0);  // non-blocking; drops if queue full
}

bool sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t val) {
    I2cWriteCmd cmd = {addr, reg, val};
    if (!xQueueSend(s_i2c_write_queue, &cmd, pdMS_TO_TICKS(100))) return false;
    if (!xSemaphoreTake(s_i2c_write_done, pdMS_TO_TICKS(200))) return false;
    return s_i2c_write_result;
}
```

- [ ] **Step 2: Verify firmware compiles**

```bash
cd /path/to/bark-buddy && pio run
```

Expected: compiles without errors. `sensor_task_start` etc. will be unresolved until main.cpp is updated — that's fine as long as it links when fully assembled. If `pio run` fails due to unresolved symbols at this stage, add a temporary stub call in main.cpp (`sensor_task_start();`) and remove it in Task 5.

- [ ] **Step 3: Commit**

```bash
git add firmware/src/sensor_task.cpp
git commit -m "feat(firmware): implement FreeRTOS sensor task"
```

---

## Task 4: Create `command_handlers.h` and `command_handlers.cpp`

Extract all handler logic from `main.cpp`. The `handle_message` if/else chain is replaced by a flat dispatch table. Handler-owned state moves here; three read-only accessors let `main.cpp`'s loop check the state it needs.

**Files:**
- Create: `firmware/include/command_handlers.h`
- Create: `firmware/src/command_handlers.cpp`

- [ ] **Step 1: Create command_handlers.h**

```cpp
// firmware/include/command_handlers.h
#pragma once
#include <ArduinoJson.h>

// Call once from setup() — initialises handler-owned state.
void handlers_init();

// Dispatch a received JSON message to the appropriate handler.
// Called from process_rx() in main.cpp.
void handle_message(const JsonDocument& doc);

// Read-only accessors for state that main loop needs.
// State is owned here; main.cpp never writes these directly.
bool          handlers_manual_servo_mode();
bool          handlers_test_mode();
unsigned long handlers_last_test_cmd();
```

- [ ] **Step 2: Create command_handlers.cpp**

This file contains: direction helpers (moved from main.cpp), all 11 handler functions, and the dispatch table.

```cpp
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
    resp["ref_type"]      = "cmd_test_mode";
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
```

- [ ] **Step 3: Verify firmware compiles**

```bash
pio run
```

Expected: compiles. There will be duplicate definitions of `direction_from_string` / `direction_to_string` until main.cpp is updated in Task 5 — if the linker errors on this, temporarily comment out those two functions in `main.cpp` to confirm the rest compiles.

- [ ] **Step 4: Commit**

```bash
git add firmware/include/command_handlers.h firmware/src/command_handlers.cpp
git commit -m "feat(firmware): add command handler dispatch table"
```

---

## Task 5: Refactor `main.cpp`

Replace the current 542-line `main.cpp` with a slim version that uses the two new modules. Key changes:
- Remove `i2c_mutex`, `Wire.begin()`, direct IMU/sonar reads, direct `sonar_set_rgb()` calls
- Remove direction helpers and all `handle_*` functions
- Add `sensor_task_start()` + `handlers_init()` in `setup()`
- Telemetry in `loop()` reads from `sensor_snapshot_get()` instead of I2C
- WiFi: non-blocking init, reconnect logic in `loop()`
- Boot message: drops `wifi`/`wifi_ip`/`tcp_port` fields (WiFi is connecting in background; these appear in periodic `telem_status` once connected)

**Files:**
- Modify: `firmware/src/main.cpp`

- [ ] **Step 1: Replace main.cpp with the refactored version**

```cpp
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
static char   tcp_rx[MAX_MESSAGE_SIZE];
static size_t tcp_rx_pos = 0;

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

// --- Send helpers ---
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
    bool servos_ok = servos_init();
    gait_init();
    handlers_init();

#if WIFI_ENABLED
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    // No blocking wait — loop() handles connect and reconnect
#endif

    last_msg_received = millis();

    // Boot message — sensor init results are ready from the snapshot
    SensorSnapshot snap;
    sensor_snapshot_get(snap);
    JsonDocument doc;
    doc["type"]         = "boot";
    doc["imu"]          = snap.imu_ok;
    doc["sonar"]        = snap.sonar_ok;
    doc["servos"]       = servos_ok;
    doc["pins_verified"] = (bool)PINS_VERIFIED;
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
        gait_set_state(GaitState::STOP);
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

    // Test mode heartbeat — exit if host goes quiet
    if (handlers_test_mode() &&
        (now - handlers_last_test_cmd() > TEST_HEARTBEAT_MS)) {
        // Re-dispatch a synthetic disable to reuse handler logic
        JsonDocument doc;
        doc["type"]   = MSG_CMD_TEST_MODE;
        doc["enable"] = false;
        handle_message(doc);
    }

    // Frail mode duty cycle
    if (servos_update_duty(now)) {
        if ((now / 500) % 2 == 0) {
            sensor_led_set(1, LED_BRIGHTNESS, LED_BRIGHTNESS / 2, 0);  // amber flash
            sensor_led_set(2, LED_BRIGHTNESS, LED_BRIGHTNESS / 2, 0);
        }
    }

    // Gait tick (skip during manual servo mode)
    if (!handlers_manual_servo_mode() && now - last_gait >= 1000 / GAIT_UPDATE_HZ) {
        if (!low_battery) gait_update(now);
        last_gait = now;
    }
}
```

- [ ] **Step 2: Build firmware**

```bash
pio run
```

Expected: compiles and links with zero errors or warnings. If there are duplicate symbol errors (`direction_from_string`, `direction_to_string`) ensure those definitions are removed from `main.cpp` — they now live only in `command_handlers.cpp`.

- [ ] **Step 3: Run host tests (regression check)**

```bash
make -C firmware/test && \
  firmware/test/test_ik && \
  firmware/test/test_transform && \
  firmware/test/test_balance && \
  firmware/test/test_offsets && \
  firmware/test/test_gait_ik
```

Expected: all tests pass. These test pure-logic headers (ik.h, balance.h, etc.) that were not touched.

- [ ] **Step 4: Commit**

```bash
git add firmware/src/main.cpp
git commit -m "refactor(firmware): slim main.cpp — sensor task, handler table, non-blocking WiFi"
```

---

## Task 6: Flash and verify on hardware

Confirm the refactored firmware behaves identically to the original on the actual robot.

**Prerequisites:** ESP32 reachable via USB serial. Python host running.

- [ ] **Step 1: Flash firmware**

```bash
pio run --target upload
```

Expected: upload succeeds, device resets.

- [ ] **Step 2: Verify boot message arrives quickly**

Open serial monitor:
```bash
pio device monitor --baud 115200
```

Expected: `{"type":"boot","imu":true,"sonar":true,"servos":true,"pins_verified":1}` appears within ~200ms of power-on (previously up to 10.5s due to WiFi blocking). The `wifi`/`wifi_ip`/`tcp_port` fields are no longer in the boot message — they appear in the first `telem_status` after WiFi connects.

- [ ] **Step 3: Verify WiFi connects in background**

Watch serial output. Within ~5s of boot, expect:
- LEDs go amber (WiFi connected, waiting for TCP client)
- First `telem_status` includes `"wifi":true,"wifi_ip":"..."` 

Serial continues responding to commands immediately — no 10s stall.

- [ ] **Step 4: Verify all commands work**

Run the servo test tool (safe, uses frail mode):
```bash
python host/servo_test.py
```

Expected: test mode enters cleanly, servo commands ack with correct pulse widths, test mode exits cleanly.

Send a ping manually over serial to confirm dispatch (replace `PORT` with your device, e.g. `/dev/tty.usbserial-0001`):
```bash
PORT=/dev/tty.usbserial-0001
echo '{"type":"ping"}' | python -c "import serial, sys; s=serial.Serial('$PORT', 115200); s.write(sys.stdin.buffer.read() + b'\n'); import time; time.sleep(0.1); print(s.read(s.in_waiting).decode())"
```

Expected: `{"type":"pong"}` response.

- [ ] **Step 5: Commit final verification note**

```bash
git commit --allow-empty -m "chore(firmware): verified refactored firmware on hardware"
```

---

## Behaviour Changes Summary

| Before | After |
|---|---|
| Boot blocks up to 10.5s (WiFi wait) | Boot completes in ~200ms |
| `boot` message includes `wifi`/`wifi_ip`/`tcp_port` | Those fields appear in `telem_status` only |
| WiFi drop = unrecoverable until reboot | WiFi auto-reconnects via `WiFi.reconnect()` |
| New command = add an if/else branch in `handle_message` | New command = one function + one table entry |
| Adding I2C sensor = edit main loop, manage mutex | Adding I2C sensor = edit sensor task only |
| `i2c_mutex` guards shared bus from two contexts | No I2C mutex needed — sensor task owns bus entirely |
