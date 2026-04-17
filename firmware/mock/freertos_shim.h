#pragma once
#include <mutex>
#include <thread>
#include <chrono>
#include <cstdint>
#include <functional>

using SemaphoreHandle_t = std::mutex*;
using TaskHandle_t      = std::thread*;
using BaseType_t        = int;
using UBaseType_t       = unsigned int;
using TickType_t        = uint32_t;

#define pdTRUE  1
#define pdFALSE 0
#define portMAX_DELAY 0xFFFFFFFFu
#define pdMS_TO_TICKS(ms) (ms)

inline SemaphoreHandle_t xSemaphoreCreateMutex() { return new std::mutex; }
inline BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t) { s->lock(); return pdTRUE; }
inline BaseType_t xSemaphoreGive(SemaphoreHandle_t s) { s->unlock(); return pdTRUE; }

inline BaseType_t xTaskCreate(void(*fn)(void*), const char*, uint32_t,
                               void* arg, UBaseType_t, TaskHandle_t*) {
    auto* t = new std::thread(fn, arg);
    t->detach();
    return pdTRUE;
}

inline void vTaskDelay(TickType_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Queue stubs (mock doesn't use LED/I2C queue).
// Define TEST_COMMS_OUT_QUEUE_STUBS before including this header to suppress
// these stubs and provide your own (e.g. in test_comms_out.cpp).
using QueueHandle_t = void*;
#ifndef TEST_COMMS_OUT_QUEUE_STUBS
inline QueueHandle_t xQueueCreate(int, int) { return nullptr; }
inline BaseType_t xQueueSend(QueueHandle_t, const void*, TickType_t) { return pdFALSE; }
inline BaseType_t xQueueReceive(QueueHandle_t, void*, TickType_t) { return pdFALSE; }
#endif
