// Mock firmware entry point.
// Parses --tcp-port <port> and --state-dir <path>, sets env vars,
// then calls the real firmware setup() / loop() in the main thread
// while a physics tick thread runs at ~200 Hz.
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <thread>
#include <chrono>
#include <atomic>

#include <ArduinoJson.h>
#include "physics.h"
#include "net_tcp.h"
#include "sensor_task.h"
#include "servos.h"
#include "config.h"

// Declared by the real firmware (main.cpp compiles in).
extern void setup();
extern void loop();

static std::atomic<bool> s_running{true};

static void physics_thread() {
    using namespace std::chrono;
    auto next = steady_clock::now();
    auto prev = next;
    while (s_running.load()) {
        next += microseconds(5000);  // 200 Hz
        auto now = steady_clock::now();
        float dt = duration<float>(now - prev).count();
        prev = now;
        physics::tick(dt);
        std::this_thread::sleep_until(next);
    }
}

// Re-sends the boot message over TCP when a client (re-)connects.
// Mirrors the boot block at the end of setup() in main.cpp.
static void send_boot_to_new_client() {
    SensorSnapshot snap;
    sensor_snapshot_get(snap);
    JsonDocument doc;
    doc["type"]          = "boot";
    doc["imu"]           = snap.imu_ok;
    doc["sonar"]         = snap.sonar_ok;
    doc["servos"]        = servos_active();
    doc["pins_verified"] = (bool)PINS_VERIFIED;
    doc["fw_version"]    = FW_VERSION;
    doc["fw_build"]      = FW_BUILD_TIMESTAMP;
    char buf[256];
    size_t n = serializeJson(doc, buf, sizeof(buf));
    buf[n] = '\n';
    net_tcp::send(buf, n + 1);
}

int main(int argc, char* argv[]) {
    int tcp_port = 9001;
    const char* state_dir = nullptr;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--tcp-port") == 0 && i + 1 < argc) {
            tcp_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--state-dir") == 0 && i + 1 < argc) {
            state_dir = argv[++i];
        }
    }

    // Communicate port to WiFi.h shim via env var (read in WiFiServer::begin()).
    char port_str[16];
    snprintf(port_str, sizeof(port_str), "%d", tcp_port);
    setenv("BARK_MOCK_TCP_PORT", port_str, 1);

    if (state_dir) {
        setenv("BARK_MOCK_STATE_DIR", state_dir, 1);
    }

    physics::init();

    std::thread phys(physics_thread);
    phys.detach();

    setup();

    while (true) {
        loop();
        if (net_tcp::client_just_connected()) {
            send_boot_to_new_client();
        }
    }
}
