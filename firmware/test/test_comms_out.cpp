// test_comms_out.cpp — Unit tests for the outbound JSON queue (comms_out.cpp).
//
// Strategy: stub comms_write_line() and include comms_out.cpp directly, on top
// of the mock's FreeRTOS queue shim — the same queue the mock firmware runs on.
// No FreeRTOS or WiFi; pure logic test.

#include "mock_arduino.h"

// --- ArduinoJson (required by comms_out.h / comms_out.cpp) ---
#include <ArduinoJson.h>

#include "freertos_shim.h"

// ---------------------------------------------------------------------------
// comms_write_line stub — captures output for verification
// ---------------------------------------------------------------------------
#include <vector>
#include <string>

static std::vector<std::string> g_written_lines;

void comms_write_line(const char* line) {
    g_written_lines.emplace_back(line);
}

static void written_lines_reset() {
    g_written_lines.clear();
}

// ---------------------------------------------------------------------------
// Now pull in the real comms_out implementation. comms_write_line is already
// defined, satisfying the call in comms_out_drain().
// ---------------------------------------------------------------------------
#include "../src/comms_out.cpp"

// ---------------------------------------------------------------------------
// Test harness
// ---------------------------------------------------------------------------
#include <cstdio>
#include <cstdlib>

static int g_pass = 0;
static int g_fail = 0;

static void check(bool cond, const char* label) {
    if (cond) {
        printf("  PASS  %s\n", label);
        g_pass++;
    } else {
        printf("  FAIL  %s\n", label);
        g_fail++;
    }
}

static void reset_all() {
    written_lines_reset();
    comms_out_init();  // fresh, empty queue
}

// ---------------------------------------------------------------------------
// Test: FIFO order — enqueue 3 items, drain, assert order
// ---------------------------------------------------------------------------
static void test_fifo_order() {
    printf("\nTest: fifo_order\n");
    reset_all();

    const char* msgs[] = { "alpha", "beta", "gamma" };
    for (int i = 0; i < 3; i++) {
        JsonDocument doc;
        doc["v"] = msgs[i];
        comms_emit_json_from_task(doc);
    }

    comms_out_drain();

    check(g_written_lines.size() == 3, "3 lines written after drain");
    if (g_written_lines.size() == 3) {
        // Each serialized line contains the value string
        check(g_written_lines[0].find("alpha") != std::string::npos, "line[0] is alpha");
        check(g_written_lines[1].find("beta")  != std::string::npos, "line[1] is beta");
        check(g_written_lines[2].find("gamma") != std::string::npos, "line[2] is gamma");
    }
}

// ---------------------------------------------------------------------------
// Test: full-queue drop — enqueue 9 items into depth-8 queue, 9th is dropped
// ---------------------------------------------------------------------------
static void test_full_queue_drop() {
    printf("\nTest: full_queue_drop\n");
    reset_all();

    for (int i = 0; i < 9; i++) {
        JsonDocument doc;
        doc["i"] = i;
        comms_emit_json_from_task(doc);
    }

    comms_out_drain();

    check(g_written_lines.size() == 8, "exactly 8 items drained (9th was dropped)");
    // The first 8 items (i=0..7) should be present; i=8 was dropped.
    bool found_8 = false;
    for (const auto& line : g_written_lines) {
        if (line.find("\"i\":8") != std::string::npos ||
            line.find("\"i\": 8") != std::string::npos) {
            found_8 = true;
        }
    }
    check(!found_8, "item i=8 (9th) is not in output (was dropped)");
}

// ---------------------------------------------------------------------------
// Test: buffer bounds — JSON > 192 bytes is truncated, not overflowed
// ---------------------------------------------------------------------------
static void test_buffer_bounds() {
    printf("\nTest: buffer_bounds\n");
    reset_all();

    // Build a JSON document that serializes to more than 192 bytes.
    // A string value of 200 'x' characters will comfortably exceed the limit.
    JsonDocument doc;
    doc["type"] = "telem_test";
    char big[210];
    memset(big, 'x', 209);
    big[209] = '\0';
    doc["data"] = big;

    comms_emit_json_from_task(doc);
    comms_out_drain();

    check(g_written_lines.size() == 1, "one item drained");
    if (!g_written_lines.empty()) {
        const std::string& line = g_written_lines[0];
        // The buf is 192 bytes; ArduinoJson serializeJson truncates and
        // null-terminates within the buffer, so strlen <= 191.
        check(line.size() <= 191, "serialized output is <= 191 chars (truncated)");
        // Confirm the buffer wasn't null-terminated past index 191 by checking
        // there's no content beyond index 191 (the string itself is bounded).
        check(line.size() > 0, "output is non-empty (partial JSON present)");
    }
}

// ---------------------------------------------------------------------------
// Test: drain on empty queue does nothing
// ---------------------------------------------------------------------------
static void test_drain_empty() {
    printf("\nTest: drain_empty\n");
    reset_all();

    comms_out_drain();
    check(g_written_lines.empty(), "drain of empty queue writes nothing");
}

// ---------------------------------------------------------------------------
// Test: multiple drain cycles work correctly
// ---------------------------------------------------------------------------
static void test_multiple_drain_cycles() {
    printf("\nTest: multiple_drain_cycles\n");
    reset_all();

    // First cycle
    JsonDocument doc1;
    doc1["cycle"] = 1;
    comms_emit_json_from_task(doc1);
    comms_out_drain();
    check(g_written_lines.size() == 1, "first drain yields 1 item");

    written_lines_reset();

    // Second cycle
    JsonDocument doc2;
    doc2["cycle"] = 2;
    comms_emit_json_from_task(doc2);
    comms_out_drain();
    check(g_written_lines.size() == 1, "second drain yields 1 item");
    check(g_written_lines[0].find("2") != std::string::npos, "second item contains cycle=2");
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main() {
    printf("=== test_comms_out ===\n");

    test_fifo_order();
    test_full_queue_drop();
    test_buffer_bounds();
    test_drain_empty();
    test_multiple_drain_cycles();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
