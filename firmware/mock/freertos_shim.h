#pragma once
#include <mutex>
#include <thread>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <vector>

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

// Bounded FIFO with FreeRTOS copy semantics. The outbound JSON queue
// (comms_out.cpp) depends on it: with no-op stubs every message sent through
// comms_emit_json_from_task() — telem_joints included — vanished in the mock.
// Never blocks; every caller in the mock build passes a zero timeout.
struct MockQueue {
    std::mutex                       mtx;
    std::deque<std::vector<uint8_t>> items;
    size_t                           depth;
    size_t                           item_size;
};
using QueueHandle_t = MockQueue*;

inline QueueHandle_t xQueueCreate(size_t depth, size_t item_size) {
    auto* q = new MockQueue;
    q->depth     = depth;
    q->item_size = item_size;
    return q;
}

inline BaseType_t xQueueSend(QueueHandle_t q, const void* item, TickType_t) {
    std::lock_guard<std::mutex> lk(q->mtx);
    if (q->items.size() >= q->depth) return pdFALSE;
    const auto* bytes = static_cast<const uint8_t*>(item);
    q->items.emplace_back(bytes, bytes + q->item_size);
    return pdTRUE;
}

inline BaseType_t xQueueReceive(QueueHandle_t q, void* item, TickType_t) {
    std::lock_guard<std::mutex> lk(q->mtx);
    if (q->items.empty()) return pdFALSE;
    std::memcpy(item, q->items.front().data(), q->item_size);
    q->items.pop_front();
    return pdTRUE;
}
