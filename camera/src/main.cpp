// Vision module firmware: one fresh JPEG per GET /capture.
//
// Runs on the Hiwonder ESP32-S3 AI Vision Module mounted on the MechDog. The
// host pulls stills from it over WiFi (host/camera.py); nothing else talks to
// it, and it has no link to the main controller. Pin map and sensor
// orientation follow Hiwonder's own firmware for this module (docs/hardware.md).

#include <Arduino.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <img_converters.h>

// Same credentials file as the main firmware.
#if __has_include("../../firmware/include/config_local.h")
#include "../../firmware/include/config_local.h"
#else
#error "WiFi credentials missing: copy firmware/include/config_local.h.example to config_local.h"
#endif

static const char* const HOSTNAME = "bark-cam";  // http://bark-cam.local/capture

// 800x600 is more than the reconstruction model reads (it downsizes to ~500 px)
// and keeps a JPEG near 50 kB, one WiFi burst.
static const framesize_t FRAME_SIZE   = FRAMESIZE_SVGA;
static const int         JPEG_QUALITY = 10;  // sensor scale, 0–63, lower is better
static const uint8_t     SOFT_JPEG_QUALITY = 85;  // encoder scale, 0–100, higher is better

static WebServer server(80);

static bool camera_start(pixformat_t format) {
    camera_config_t c = {};
    c.ledc_channel = LEDC_CHANNEL_0;
    c.ledc_timer   = LEDC_TIMER_0;
    c.pin_d0       = 11;
    c.pin_d1       = 9;
    c.pin_d2       = 8;
    c.pin_d3       = 10;
    c.pin_d4       = 12;
    c.pin_d5       = 18;
    c.pin_d6       = 17;
    c.pin_d7       = 16;
    c.pin_xclk     = 15;
    c.pin_pclk     = 13;
    c.pin_vsync    = 6;
    c.pin_href     = 7;
    c.pin_sccb_sda = 4;
    c.pin_sccb_scl = 5;
    c.pin_pwdn     = -1;
    c.pin_reset    = -1;
    c.xclk_freq_hz = 15000000;
    c.frame_size   = FRAME_SIZE;
    c.pixel_format = format;
    c.fb_location  = CAMERA_FB_IN_PSRAM;
    c.fb_count     = 2;
    // Keep capturing and hand out the newest frames, so a still requested after
    // the dog stops was exposed after it stopped, not left over from the walk.
    c.grab_mode    = CAMERA_GRAB_LATEST;
    c.jpeg_quality = JPEG_QUALITY;
    return esp_camera_init(&c) == ESP_OK;
}

// Mounted orientation per sensor, as in Hiwonder's firmware, so images are upright.
static void orient_sensor() {
    sensor_t* s = esp_camera_sensor_get();
    switch (s->id.PID) {
        case OV2640_PID:
        case OV3660_PID:
        case GC032A_PID:
            s->set_vflip(s, 1);
            break;
        case GC0308_PID:
            s->set_hmirror(s, 0);
            break;
    }
}

static void handle_capture() {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        server.send(503, "text/plain", "capture failed\n");
        return;
    }
    server.sendHeader("Cache-Control", "no-store");
    if (fb->format == PIXFORMAT_JPEG) {
        server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
    } else {
        uint8_t* jpeg = nullptr;
        size_t   len  = 0;
        if (frame2jpg(fb, SOFT_JPEG_QUALITY, &jpeg, &len)) {
            server.send_P(200, "image/jpeg", (const char*)jpeg, len);
        } else {
            server.send(503, "text/plain", "jpeg encode failed\n");
        }
        free(jpeg);
    }
    esp_camera_fb_return(fb);
}

static void handle_root() {
    sensor_t* s = esp_camera_sensor_get();
    char body[160];
    snprintf(body, sizeof(body),
             "%s\nsensor PID 0x%02x, %s\nWiFi RSSI %d dBm\nGET /capture for a JPEG still\n",
             HOSTNAME, s->id.PID,
             s->pixformat == PIXFORMAT_JPEG ? "sensor JPEG" : "RGB565 + software JPEG",
             (int)WiFi.RSSI());
    server.send(200, "text/plain", body);
}

void setup() {
    Serial.begin(115200);

    // Sensors without an on-chip JPEG encoder reject the first init.
    if (!camera_start(PIXFORMAT_JPEG)) {
        esp_camera_deinit();
        if (!camera_start(PIXFORMAT_RGB565)) {
            Serial.println("camera init failed");
            return;
        }
    }
    orient_sensor();

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // modem sleep adds 100+ ms to each request
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("joining %s", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        Serial.print(".");
    }
    Serial.printf("\nhttp://%s/capture\n", WiFi.localIP().toString().c_str());
    MDNS.begin(HOSTNAME);

    server.on("/", handle_root);
    server.on("/capture", handle_capture);
    server.begin();
}

void loop() {
    server.handleClient();
    delay(2);
}
