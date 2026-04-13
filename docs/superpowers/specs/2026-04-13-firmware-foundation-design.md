# Firmware Foundation Refactor — Design Spec
**Date:** 2026-04-13
**Status:** Approved

## Goal

Establish a clean, extensible firmware foundation for the Bark-Buddy custom ESP32 firmware — optimized for future additions (more I2C devices, new command types, claw/camera hardware) with minimal code complexity. Three targeted changes: sensor task, handler dispatch table, WiFi reconnect.

---

## Architecture

Two concurrent execution contexts:

```
┌─────────────────────────────────┐   ┌─────────────────────────────────┐
│         Sensor Task             │   │         Main Loop               │
│  (FreeRTOS, ~4KB stack)         │   │  (Arduino loop)                 │
│                                 │   │                                 │
│  owns I2C bus entirely          │   │  serial/TCP RX → dispatch       │
│  reads IMU @ 50Hz               │   │  reads SensorSnapshot           │
│  reads sonar @ 20Hz             │   │  sends telemetry                │
│  drains LED queue               │   │  ticks gait engine              │
│  writes → SensorSnapshot        │   │  WiFi reconnect check           │
└──────────────┬──────────────────┘   └──────────────┬──────────────────┘
               │  mutex-protected                     │
               └──────────── SensorSnapshot ──────────┘
                             + LED queue (1 slot)
```

The existing `i2c_mutex` in `main.cpp` is removed — only the sensor task touches I2C, so no mutex is needed there. The snapshot mutex is the only shared-memory guard.

---

## Components

### 1. Sensor Task (`sensor_task.cpp/.h`)

Owns the I2C bus. Reads sensors at their configured rates, writes results to a shared `SensorSnapshot` struct under a mutex. Drains a LED command queue each cycle.

```cpp
struct SensorSnapshot {
    float pitch, roll, yaw;
    float ax, ay, az, gx, gy, gz;
    uint16_t sonar_mm;
    bool imu_ok;
    bool sonar_ok;
};

struct LedCmd { uint8_t led, r, g, b; };

// Starts the FreeRTOS task. Blocks until the task completes its first init
// pass (≤50ms), then returns. imu_ok/sonar_ok in the snapshot are valid
// immediately after this call returns — used by setup() for the boot message.
void sensor_task_start();
void sensor_snapshot_get(SensorSnapshot& out);

// Posts a LED command. Uses xQueueOverwrite() — latest write always wins,
// never blocks. Safe to call from main loop or command handlers.
void sensor_led_set(uint8_t led, uint8_t r, uint8_t g, uint8_t b);
```

Task loop: run init (Wire.begin, imu_init, sonar_init) → signal ready semaphore → read IMU if due (50Hz) → read sonar if due (20Hz) → drain LED queue → `vTaskDelay(1ms)`.

`Wire.begin()` moves from `setup()` into the sensor task's init pass. `setup()` calls `sensor_task_start()`, which blocks on the ready semaphore, then reads `imu_ok`/`sonar_ok` from the snapshot for the boot message. Boot sequence and boot message content are unchanged.

**Extending:** adding a new I2C device means adding its read to the sensor task and its fields to `SensorSnapshot`. No changes to `main.cpp` or command handlers.

### 2. Handler Dispatch Table (`command_handlers.cpp/.h`)

Replaces the if/else `strcmp` chain in `handle_message()` with a flat array of `{type, fn}` pairs. `handle_message()` never changes again — new commands require only a new function and one table entry.

```cpp
typedef void (*HandlerFn)(const JsonDocument&);
struct Handler { const char* type; HandlerFn fn; };

static const Handler k_handlers[] = {
    { MSG_PING,            handle_ping            },
    { MSG_CMD_MOVE,        handle_cmd_move        },
    { MSG_CMD_STAND,       handle_cmd_stand       },
    { MSG_CMD_BALANCE,     handle_cmd_balance     },
    { MSG_CMD_SERVO,       handle_cmd_servo       },
    { MSG_CMD_LED,         handle_cmd_led         },
    { MSG_CMD_TRANSFORM,   handle_cmd_transform   },
    { MSG_CMD_GAIT_PARAMS, handle_cmd_gait_params },
    { MSG_CMD_TEST_MODE,   handle_cmd_test_mode   },
    { MSG_CMD_OFFSET,      handle_cmd_offset      },
    { MSG_CMD_I2C_WRITE,   handle_cmd_i2c_write   },
};

void handle_message(const JsonDocument& doc) {
    const char* type = doc["type"];
    if (!type) return;
    for (const auto& h : k_handlers) {
        if (strcmp(type, h.type) == 0) { h.fn(doc); return; }
    }
    send_ack(type, false, "unknown_type");
}
```

Handler functions are the existing if/else bodies lifted out verbatim — no logic changes.

`handlers_init()` called once from `setup()` to initialize any handler-owned state (e.g. the test mode and manual servo mode flags, currently globals in `main.cpp`, move here).

### 3. WiFi Reconnect (`main.cpp`)

`setup()` kicks off WiFi without blocking:

```cpp
WiFi.mode(WIFI_STA);
WiFi.begin(WIFI_SSID, WIFI_PASS);
// no while() — loop() handles connect/reconnect
```

`loop()` checks WiFi state each iteration:

```cpp
bool now_connected = (WiFi.status() == WL_CONNECTED);
if (now_connected && !wifi_connected) {
    wifi_connected = true;
    tcp_server.begin();
    tcp_server.setNoDelay(true);
    sensor_led_set(1, LED_BRIGHTNESS, LED_BRIGHTNESS/2, 0);  // amber
    sensor_led_set(2, LED_BRIGHTNESS, LED_BRIGHTNESS/2, 0);
}
if (!now_connected && wifi_connected) {
    wifi_connected = false;
    tcp_client.stop();
    WiFi.reconnect();
}
```

Boot is instant. Serial works immediately. WiFi comes up in the background and recovers automatically if dropped.

---

## File Structure

| File | Responsibility | Target size |
|---|---|---|
| `firmware/src/main.cpp` | `setup()`, `loop()`, `send_json()`, `process_rx()`, WiFi state | ~150 lines |
| `firmware/src/command_handlers.cpp` | Handler table + all `handle_*` functions | ~300 lines |
| `firmware/include/command_handlers.h` | Public interface: `handlers_init()`, `handle_message()` | ~20 lines |
| `firmware/src/sensor_task.cpp` | FreeRTOS task, snapshot mutex, LED queue | ~120 lines |
| `firmware/include/sensor_task.h` | `SensorSnapshot`, `LedCmd`, public API | ~30 lines |

All other files (`gait.cpp`, `imu.cpp`, `servos.cpp`, `balance.cpp`, `offsets.cpp`, `sonar.cpp`) are untouched.

---

## Data Flow (after refactor)

```
boot:
  sensor_task_start()     → FreeRTOS task created, I2C init inside task
  handlers_init()         → command handler state initialized
  WiFi.begin() (no block)
  send boot JSON

loop():
  WiFi reconnect check
  serial/TCP RX → process_rx() → handle_message() → handler table
  heartbeat watchdog
  battery ADC read + telem
  sensor_snapshot_get() → gait_update_imu() + IMU telem
  sonar telem (from snapshot)
  status telem
  gait_update()
```

---

## What Is Not Changed

- Protocol: all message types, field names, and semantics are identical
- Gait engine, IK, balance, offsets, servos — untouched
- Telemetry rates and heartbeat timeout — unchanged
- Test mode and frail mode behavior — unchanged (state moves to `command_handlers.cpp`)
- Serial always works; WiFi is additive

---

## Testing

- `pio test` suite must pass before and after
- Boot status message (`{"type":"boot",...}`) must appear within 100ms of power-on (vs. up to 10.5s now)
- WiFi connects in background — verified by checking amber LED appears within ~5s without blocking serial
- All existing commands verified via `host/servo_test.py` and manual testing
