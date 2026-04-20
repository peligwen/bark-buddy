#pragma once
// Drop-in <WiFi.h> shim for mock build. Delegates TCP to net_tcp.
#include "net_tcp.h"
#include <cstdlib>
#include <cstring>
#include <string>

#define WL_CONNECTED 3
#define WIFI_STA 1

struct IPAddress {
    std::string _s;
    explicit IPAddress(const char* s = "127.0.0.1") : _s(s) {}
    std::string toString() const { return _s; }
    operator std::string() const { return _s; }
    const char* c_str() const { return _s.c_str(); }
};

struct WiFiClass {
    int status() { return WL_CONNECTED; }
    IPAddress localIP() { return IPAddress("127.0.0.1"); }
    bool softAP(const char*, const char* = nullptr) { return true; }
    void begin(const char*, const char*) {}
    void disconnect(bool = false) {}
    void reconnect() {}
    void mode(int) {}
};
extern WiFiClass WiFi;

struct WiFiClient {
    bool _valid = false;

    WiFiClient() : _valid(false) {}

    // Truthy when we have a live connection.
    explicit operator bool() const { return _valid && net_tcp::is_connected(); }
    bool connected()       { return _valid && net_tcp::is_connected(); }
    bool available()       { return _valid && net_tcp::data_available(); }
    int  read()            { return _valid ? net_tcp::read_byte() : -1; }

    // OTA path — not supported in mock; always returns 0.
    size_t readBytes(uint8_t*, size_t) { return 0; }

    IPAddress remoteIP() { return IPAddress("127.0.0.1"); }

    size_t print(const char* s) {
        return net_tcp::send(s, strlen(s)) ? strlen(s) : 0;
    }
    size_t println(const char* s = "") {
        size_t n = net_tcp::send(s, strlen(s));
        net_tcp::send("\n", 1);
        return n + 1;
    }
    // ArduinoJson uses stream-style write.
    size_t write(uint8_t c) { return net_tcp::send((const char*)&c, 1) ? 1 : 0; }
    size_t write(const uint8_t* buf, size_t n) {
        return net_tcp::send((const char*)buf, n) ? n : 0;
    }

    // Used by net_tcp readline path (legacy; host transport uses the callback path)
    int readBytesUntil(char, char* buf, int len) {
        return net_tcp::readline(buf, len);
    }

    void setNoDelay(bool) {}
    void stop() {}
};

struct WiFiServer {
    int _default_port;
    explicit WiFiServer(int port = 9000) : _default_port(port) {}
    void begin() {
        const char* env = getenv("BARK_MOCK_TCP_PORT");
        int port = (env && *env) ? atoi(env) : _default_port;
        net_tcp::init(port);
    }
    void setNoDelay(bool) {}

    // Returns a valid WiFiClient if a new connection arrived, else invalid one.
    // available() is the 2.x name; accept() is the 3.x name.
    WiFiClient available() {
        WiFiClient c;
        c._valid = net_tcp::accept_client();
        return c;
    }
    WiFiClient accept() { return available(); }
};
