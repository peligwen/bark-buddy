// firmware/src/sensor_task.cpp
#include "sensor_task.h"
#include "imu.h"
#include "sonar.h"
#include "button.h"
#include "config.h"
#include "protocol.h"
#include "comms.h"
#include "gpio_aux.h"
#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

#ifndef MOCK_FIRMWARE
#include <freertos/portmacro.h>
#else
#include <mutex>
#endif

static SensorSnapshot    s_snapshot       = {};
static SemaphoreHandle_t s_snapshot_mutex = nullptr;
static SemaphoreHandle_t s_ready_sem      = nullptr;
static QueueHandle_t     s_led_queue      = nullptr;
static QueueHandle_t     s_i2c_cmd_queue  = nullptr;
static SemaphoreHandle_t s_i2c_done       = nullptr;
static I2cResult         s_i2c_result     = {};

// Binary semaphore given by the INT2 ISR (hardware) or sensor_imu_signal_ready()
// (mock). sensor_task_fn blocks on this with a 50ms safety timeout so that
// polling still works if INT2 is unconfigured or interrupts stall.
static SemaphoreHandle_t s_imu_rdy = nullptr;

// GPIO subscription state — protected by s_gpio_mux.
struct GpioSub { uint8_t pin; GpioMode mode; };
static GpioSub  s_gpio_subs[4]  = {};
static uint8_t  s_gpio_sub_count = 0;

#ifndef MOCK_FIRMWARE
static portMUX_TYPE s_gpio_mux = portMUX_INITIALIZER_UNLOCKED;
#define GPIO_SUB_ENTER_CRITICAL() portENTER_CRITICAL(&s_gpio_mux)
#define GPIO_SUB_EXIT_CRITICAL()  portEXIT_CRITICAL(&s_gpio_mux)
#else
static std::mutex s_gpio_mutex;
#define GPIO_SUB_ENTER_CRITICAL() s_gpio_mutex.lock()
#define GPIO_SUB_EXIT_CRITICAL()  s_gpio_mutex.unlock()
#endif

#ifndef MOCK_FIRMWARE
static void IRAM_ATTR imu_isr() {
    BaseType_t higher_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_imu_rdy, &higher_woken);
    portYIELD_FROM_ISR(higher_woken);
}
#endif

static void sensor_task_fn(void*) {
    // Take ownership of the I2C bus. Short timeout prevents sonar read stalls
    // from blocking the task for >1s on the rare occasion the module is busy.
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
#if I2C2_ENABLED
    Wire1.begin(I2C2_SDA_PIN, I2C2_SCL_PIN, I2C2_FREQ);
#endif

    button_init();

#ifndef MOCK_FIRMWARE
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    digitalWrite(ONBOARD_LED_PIN, HIGH);  // HIGH = off (active-LOW)
#endif

    bool imu_ok   = imu_init(Wire);
    bool sonar_ok = sonar_init(Wire);

#ifndef MOCK_FIRMWARE
    // Attach interrupt on INT2 data-ready line (GPIO 35, input-only pin).
    // Must be done after imu_init() so the QMI8658 is already configured
    // to assert INT2 on data-ready (CTRL8=0x40 set in imu_init).
    pinMode(IMU_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imu_isr, RISING);
#endif

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

    unsigned long last_sonar = 0;

    for (;;) {
        // Block until IMU data-ready interrupt fires (or 50ms safety timeout).
        // On hardware: GPIO 35 INT2 RISING edge gives the semaphore via imu_isr().
        // On mock:     sensor_imu_signal_ready() gives it from the 50Hz IMU thread.
        // Safety timeout (TELEM_IMU_HZ period + 30ms slack) ensures polling continues
        // even if INT2 is not configured or interrupts stall.
        const TickType_t imu_timeout = pdMS_TO_TICKS(1000 / TELEM_IMU_HZ + IMU_INT_SLACK_MS);  // 20ms IMU period + 30ms ISR wake slack
        xSemaphoreTake(s_imu_rdy, imu_timeout);
        // Read IMU regardless of whether the semaphore fired or we timed out;
        // the safety fallback preserves the original polling behaviour.
        {
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
        }

        unsigned long now = millis();

        // Read sonar at 10Hz — module needs ~60ms for a no-object measurement cycle;
        // telemetry still emits at TELEM_SONAR_HZ from the snapshot.
        if (now - last_sonar >= 100) {
            uint16_t dist = sonar_read_mm();
            xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
            s_snapshot.sonar_mm = dist;
            xSemaphoreGive(s_snapshot_mutex);
            last_sonar = now;
        }

        // Drain LED queue — process all pending colour changes
        LedCmd led;
        while (xQueueReceive(s_led_queue, &led, 0)) {
            if (led.led == 0) {
                // Onboard blue LED, active-LOW
                bool on = (led.r || led.g || led.b);
#ifndef MOCK_FIRMWARE
                digitalWrite(ONBOARD_LED_PIN, on ? LOW : HIGH);
#else
                printf("[mock] onboard_led %s\n", on ? "on" : "off");
#endif
            } else {
                sonar_set_rgb(led.led, led.r, led.g, led.b);
            }
        }

        // Service a pending I2C operation (scan/read/write, bus 1 or 2)
        I2cCmd i2ccmd;
        if (xQueueReceive(s_i2c_cmd_queue, &i2ccmd, 0)) {
            I2cResult res = {};
#if I2C2_ENABLED
            TwoWire& wire = (i2ccmd.bus == 2) ? Wire1 : Wire;
#else
            TwoWire& wire = Wire;
#endif
            switch (i2ccmd.op) {
                case I2cOp::SCAN:
                    for (uint8_t a = 1; a < 127 && res.addr_count < 16; a++) {
                        wire.beginTransmission(a);
                        if (wire.endTransmission() == 0) res.addrs[res.addr_count++] = a;
                    }
                    res.ok = true;
                    break;
                case I2cOp::READ:
                    wire.beginTransmission(i2ccmd.addr);
                    wire.write(i2ccmd.reg);
                    if (wire.endTransmission(false) == 0) {
                        uint8_t got = wire.requestFrom((int)i2ccmd.addr, (int)i2ccmd.len);
                        for (uint8_t i = 0; i < got; i++) res.data[i] = wire.read();
                        res.data_len = got;
                        res.ok = (got == i2ccmd.len);
                    }
                    break;
                case I2cOp::WRITE:
                    wire.beginTransmission(i2ccmd.addr);
                    wire.write(i2ccmd.reg);
                    wire.write(i2ccmd.val);
                    res.ok = (wire.endTransmission() == 0);
                    break;
            }
            s_i2c_result = res;
            xSemaphoreGive(s_i2c_done);
        }

        // Poll button every loop pass; function handles debounce timing
        ButtonEvent btn = button_update((uint32_t)millis());
        if (btn != ButtonEvent::NONE) {
            JsonDocument ev;
            ev["type"]    = MSG_TELEM_BUTTON;
            ev["event"]   = (btn == ButtonEvent::PRESS)      ? "press"
                          : (btn == ButtonEvent::RELEASE)    ? "release"
                          :                                    "long_press";
            ev["held_ms"] = button_held_ms();
            send_json(ev);
        }

        // Poll subscribed GPIO aux pins at ~20 Hz (every 50ms)
        static uint32_t s_gpio_poll_ms = 0;
        uint32_t now_ms = (uint32_t)millis();
        if (now_ms - s_gpio_poll_ms >= 50) {
            s_gpio_poll_ms = now_ms;

            // Snapshot the subscription table under lock, then emit outside lock.
            GpioSub subs_snap[4];
            uint8_t sub_count_snap;
            GPIO_SUB_ENTER_CRITICAL();
            sub_count_snap = s_gpio_sub_count;
            for (uint8_t i = 0; i < sub_count_snap; i++) subs_snap[i] = s_gpio_subs[i];
            GPIO_SUB_EXIT_CRITICAL();

            for (uint8_t i = 0; i < sub_count_snap; i++) {
                uint8_t pin = subs_snap[i].pin;
                int digital_val = gpio_aux_read_digital(pin);
                // ADC is only valid on GPIO 32 and 33
                int analog_val  = (pin == 32 || pin == 33) ? gpio_aux_read_analog(pin) : -1;
                JsonDocument telem;
                telem["type"]    = MSG_TELEM_GPIO;
                telem["pin"]     = pin;
                telem["digital"] = digital_val;
                telem["analog"]  = analog_val;
                send_json(telem);
            }
        }
    }
}

void sensor_task_start() {
    s_snapshot_mutex = xSemaphoreCreateMutex();
    s_ready_sem      = xSemaphoreCreateBinary();
    s_imu_rdy        = xSemaphoreCreateBinary();
    s_led_queue      = xQueueCreate(4, sizeof(LedCmd));
    s_i2c_cmd_queue  = xQueueCreate(1, sizeof(I2cCmd));
    s_i2c_done       = xSemaphoreCreateBinary();

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

bool sensor_i2c_op(const I2cCmd& cmd, I2cResult& out) {
    if (!xQueueSend(s_i2c_cmd_queue, &cmd, pdMS_TO_TICKS(100))) return false;
    if (!xSemaphoreTake(s_i2c_done, pdMS_TO_TICKS(200))) return false;
    out = s_i2c_result;
    return true;
}

bool sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t val) {
    I2cCmd cmd = {};
    cmd.op   = I2cOp::WRITE;
    cmd.bus  = 1;
    cmd.addr = addr;
    cmd.reg  = reg;
    cmd.val  = val;
    I2cResult res;
    if (!sensor_i2c_op(cmd, res)) return false;
    return res.ok;
}

void sensor_imu_signal_ready() {
    if (s_imu_rdy) xSemaphoreGive(s_imu_rdy);
}

bool sensor_gpio_subscribe(uint8_t pin, GpioMode mode) {
    if (!gpio_aux_allowlisted(pin)) return false;
    GPIO_SUB_ENTER_CRITICAL();
    // Check if already subscribed — update mode in place
    for (uint8_t i = 0; i < s_gpio_sub_count; i++) {
        if (s_gpio_subs[i].pin == pin) {
            s_gpio_subs[i].mode = mode;
            GPIO_SUB_EXIT_CRITICAL();
            gpio_aux_set_mode(pin, mode);
            return true;
        }
    }
    if (s_gpio_sub_count >= 4) {
        GPIO_SUB_EXIT_CRITICAL();
        return false;
    }
    s_gpio_subs[s_gpio_sub_count++] = {pin, mode};
    GPIO_SUB_EXIT_CRITICAL();
    gpio_aux_set_mode(pin, mode);
    return true;
}

void sensor_gpio_unsubscribe(uint8_t pin) {
    GPIO_SUB_ENTER_CRITICAL();
    for (uint8_t i = 0; i < s_gpio_sub_count; i++) {
        if (s_gpio_subs[i].pin == pin) {
            // Compact: shift remaining entries down
            for (uint8_t j = i; j + 1 < s_gpio_sub_count; j++) {
                s_gpio_subs[j] = s_gpio_subs[j + 1];
            }
            s_gpio_sub_count--;
            break;
        }
    }
    GPIO_SUB_EXIT_CRITICAL();
}
