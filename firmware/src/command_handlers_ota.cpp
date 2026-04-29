// firmware/src/command_handlers_ota.cpp
//
// OTA update over WiFi. Two handlers and the helpers they need:
//
//   handle_cmd_ota_request_nonce — issue a 32-byte single-use nonce
//   handle_cmd_ota_update        — verify auth, fetch + flash the binary
//
// Auth flow (WiFi only — serial is trusted):
//   host  -> {cmd_ota_request_nonce}
//   fw    -> {telem_ota_nonce, nonce: <32B hex>}
//   host  -> {cmd_ota_update, url, sha256, nonce, sig}        sig = Ed25519(nonce ‖ sha256)
//   fw    -> verify nonce + sig, fetch URL, hash + flash, restart
//
// `sha256` is required (non-empty) on the WiFi path: the signature covers the
// sha256 bytes, so an empty hash would let any binary be substituted with a
// valid signature over an empty digest.
//
// URL host must equal the connected TCP client's IP, and may not contain '@'
// (userinfo bypass prevention).

#include "command_handlers_internal.h"
#include "comms.h"
#include "config.h"
#include "protocol.h"
#include "update_led.h"
#include "sensor_task.h"
#include "servos.h"
#include "ota_auth.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <string.h>
#include <monocypher-ed25519.h>

#if WIFI_ENABLED
#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>
#include <mbedtls/sha256.h>
#endif

// --- Hex helpers (file-local) ---

static bool hex_decode32(const char* hex, uint8_t out[32]) {
    if (!hex || strlen(hex) != 64) return false;
    for (int i = 0; i < 32; i++) {
        char b[3] = { hex[i*2], hex[i*2+1], 0 };
        char* end = nullptr;
        out[i] = (uint8_t)strtoul(b, &end, 16);
        if (end != b + 2) return false;
    }
    return true;
}

static bool hex_decode64(const char* hex, uint8_t out[64]) {
    if (!hex || strlen(hex) != 128) return false;
    for (int i = 0; i < 64; i++) {
        char b[3] = { hex[i*2], hex[i*2+1], 0 };
        char* end = nullptr;
        out[i] = (uint8_t)strtoul(b, &end, 16);
        if (end != b + 2) return false;
    }
    return true;
}

// --- Nonce request ---

void handle_cmd_ota_request_nonce(const JsonDocument& /*doc*/, MsgSource /*source*/) {
#if !WIFI_ENABLED
    send_ack(MSG_CMD_OTA_REQUEST_NONCE, false, "wifi_disabled");
#else
    uint8_t nonce[32];
    ota_nonce_issue(nonce);
    char hex[65];
    for (int i = 0; i < 32; i++) snprintf(hex + i * 2, 3, "%02x", nonce[i]);
    hex[64] = '\0';
    JsonDocument resp;
    resp["type"]  = MSG_TELEM_OTA_NONCE;
    resp["nonce"] = hex;
    send_json(resp);
#endif
}

// --- OTA update ---

void handle_cmd_ota_update(const JsonDocument& doc, MsgSource source) {
#if !WIFI_ENABLED
    (void)doc; (void)source;
    send_ack(MSG_CMD_OTA_UPDATE, false, "wifi_disabled");
    return;
#else
    const char* url             = doc["url"]    | "";
    const char* expected_sha256 = doc["sha256"] | "";
    if (!url || url[0] == '\0') {
        send_ack(MSG_CMD_OTA_UPDATE, false, "missing_url");
        return;
    }

    if (source != MsgSource::SERIAL) {
        const char* nonce_hex = doc["nonce"] | "";
        const char* sig_hex   = doc["sig"]   | "";
        if (!nonce_hex[0] || !sig_hex[0]) {
            send_ack(MSG_CMD_OTA_UPDATE, false, "missing_auth");
            return;
        }
        if (!expected_sha256 || expected_sha256[0] == '\0') {
            send_ack(MSG_CMD_OTA_UPDATE, false, "missing_sha256");
            return;
        }
        uint8_t nonce_bytes[32], sha256_bytes[32], sig_bytes[64];
        if (!hex_decode32(nonce_hex, nonce_bytes) ||
            !hex_decode32(expected_sha256, sha256_bytes) ||
            !hex_decode64(sig_hex, sig_bytes)) {
            send_ack(MSG_CMD_OTA_UPDATE, false, "bad_auth_encoding");
            return;
        }
        if (!ota_nonce_verify(nonce_bytes, sha256_bytes, sig_bytes)) {
            send_ack(MSG_CMD_OTA_UPDATE, false, "sig");
            return;
        }
    }

    // Only allow OTA downloads from the connected TCP client.
    // IPv6 bracket notation (http://[::1]/path) is not supported.
    {
        String client_ip = get_tcp_client_ip();
        if (client_ip.isEmpty()) {
            send_ack(MSG_CMD_OTA_UPDATE, false, "url_not_allowed");
            return;
        }
        const char* after_scheme = strstr(url, "://");
        if (!after_scheme) {
            send_ack(MSG_CMD_OTA_UPDATE, false, "url_not_allowed");
            return;
        }
        after_scheme += 3;
        const char* auth_end = strchr(after_scheme, '/');
        size_t auth_len = auth_end ? (size_t)(auth_end - after_scheme) : strlen(after_scheme);
        if (memchr(after_scheme, '@', auth_len) != nullptr) {
            send_ack(MSG_CMD_OTA_UPDATE, false, "url_not_allowed");
            return;
        }
        const char* end = after_scheme;
        while (*end && *end != ':' && *end != '/') end++;
        String url_host(after_scheme, (size_t)(end - after_scheme));
        if (url_host != client_ip) {
            send_ack(MSG_CMD_OTA_UPDATE, false, "url_not_allowed");
            return;
        }
    }
    send_ack(MSG_CMD_OTA_UPDATE, true);

    update_led_set_active(true);

    // Detach servos immediately — loop() can't drive ramps while this handler blocks.
    servos_detach_all();

    auto ota_cleanup = []() {
        update_led_set_active(false);
        sensor_led_set(1, 0, LED_BRIGHTNESS, 0);
        sensor_led_set(2, 0, LED_BRIGHTNESS, 0);
    };

    auto send_status = [](const char* status, const char* error = nullptr) {
        JsonDocument s;
        s["type"]   = MSG_OTA_STATUS;
        s["status"] = status;
        if (error) s["error"] = error;
        send_json(s);
    };

    send_status("downloading");

    HTTPClient http;
    http.begin(url);
    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        send_status("failed", "http_error");
        http.end();
        ota_cleanup();
        return;
    }

    int len = http.getSize();
    WiFiClient* stream = http.getStreamPtr();
    if (!Update.begin(len > 0 ? len : UPDATE_SIZE_UNKNOWN)) {
        send_status("failed", "update_begin_failed");
        http.end();
        ota_cleanup();
        return;
    }

    send_status("flashing");

    uint8_t buf[OTA_CHUNK_BYTES];
    mbedtls_sha256_context sha_ctx;
    mbedtls_sha256_init(&sha_ctx);
    mbedtls_sha256_starts(&sha_ctx, 0);  // 0 = SHA-256 (not SHA-224)

    bool write_error = false;
    bool timeout_error = false;
    size_t total_received = 0;
    unsigned long last_data = millis();
    while (http.connected()) {
        int avail = stream->available();
        if (avail <= 0) {
            if (millis() - last_data > OTA_IDLE_TIMEOUT_MS) {
                timeout_error = true;
                break;
            }
            delay(1);
            continue;
        }
        int n = stream->readBytes(buf, sizeof(buf));
        if (n <= 0) {
            delay(1);
            continue;
        }
        last_data = millis();
        total_received += n;
        size_t written = Update.write(buf, n);
        if (written != (size_t)n) {
            write_error = true;
            break;
        }
        mbedtls_sha256_update(&sha_ctx, buf, n);
    }

    uint8_t hash[32];
    int mret = mbedtls_sha256_finish(&sha_ctx, hash);
    mbedtls_sha256_free(&sha_ctx);

    http.end();

    if (write_error) {
        Update.abort();
        send_status("failed", "flash_error");
        ota_cleanup();
        return;
    }
    if (timeout_error) {
        Update.abort();
        send_status("failed", "download_timeout");
        ota_cleanup();
        return;
    }
    if (len > 0 && total_received != (size_t)len) {
        Update.abort();
        send_status("failed", "incomplete_download");
        ota_cleanup();
        return;
    }
    if (mret != 0) {
        Update.abort();
        send_status("failed", "hash_compute_error");
        ota_cleanup();
        return;
    }

    // Optional SHA-256 verification on the serial path (where it's allowed
    // to be empty). On the WiFi path we already required non-empty above.
    if (expected_sha256 && expected_sha256[0] != '\0') {
        char actual_hex[65];
        for (int i = 0; i < 32; i++) {
            snprintf(actual_hex + i * 2, 3, "%02x", hash[i]);
        }
        actual_hex[64] = '\0';
        if (strncmp(actual_hex, expected_sha256, 64) != 0) {
            Update.abort();
            send_status("failed", "hash_mismatch");
            ota_cleanup();
            return;
        }
    }

    if (Update.end() && Update.isFinished()) {
        send_status("complete");
        update_led_set_active(false);
        delay(1000);
        ESP.restart();
    } else {
        send_status("failed", "flash_error");
        ota_cleanup();
    }
#endif
}
