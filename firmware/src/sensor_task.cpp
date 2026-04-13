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
