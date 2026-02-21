#ifndef CAM_UTILS_H
#define CAM_UTILS_H

// Pin definition for AI Thinker ESP32-CAM
#define PWDN_GPIO_NUM    32
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27

#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

#include "esp_camera.h"
#include <Arduino.h>

// Configure according to your module (change if required)
bool camera_init() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;  
  // Reduced clock frequency to prevent DMA overflow
  config.xclk_freq_hz = 8000000; // Reduced from 10MHz to 8MHz
  config.pixel_format = PIXFORMAT_JPEG;
  Serial.println("[CAM] PSRAM found, using optimized settings");
  config.frame_size   = FRAMESIZE_QVGA;     // Can use larger frame with PSRAM
  config.jpeg_quality = 12;                // Better quality with PSRAM
  config.fb_count     = 1;                 // Double buffering
  config.fb_location  = CAMERA_FB_IN_PSRAM; // Use PSRAM for frame buffers

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Camera init failed: 0x%x\n", err);
    return false;
  }

  // Get camera sensor and apply additional optimizations
  sensor_t * s = esp_camera_sensor_get();
  if (s) {
    // Optimize sensor settings to reduce DMA load
    s->set_brightness(s, 0);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, etc)
    s->set_whitebal(s, 1);       // 0 = disable, 1 = enable
    s->set_awb_gain(s, 1);       // 0 = disable, 1 = enable
    s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled
    s->set_exposure_ctrl(s, 1);  // 0 = disable, 1 = enable
    s->set_aec2(s, 0);           // 0 = disable, 1 = enable
    s->set_ae_level(s, 0);       // -2 to 2
    s->set_aec_value(s, 300);    // 0 to 1200
    s->set_gain_ctrl(s, 1);      // 0 = disable, 1 = enable
    s->set_agc_gain(s, 0);       // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)0); // 0 to 6
    s->set_bpc(s, 0);            // 0 = disable, 1 = enable
    s->set_wpc(s, 1);            // 0 = disable, 1 = enable
    s->set_raw_gma(s, 1);        // 0 = disable, 1 = enable
    s->set_lenc(s, 1);           // 0 = disable, 1 = enable
    s->set_hmirror(s, 0);        // 0 = disable, 1 = enable
    s->set_vflip(s, 0);          // 0 = disable, 1 = enable
    s->set_dcw(s, 1);            // 0 = disable, 1 = enable
    s->set_colorbar(s, 0);       // 0 = disable, 1 = enable
    
    Serial.println("[CAM] Sensor optimizations applied");
  }

  Serial.println("[CAM] Camera initialized with DMA overflow prevention.");
  return true;
}

// Capture and return a camera_fb_t* (remember to call esp_camera_fb_return)
camera_fb_t* capture_image() {
  camera_fb_t* fb = esp_camera_fb_get();
  
  if (!fb) {
    Serial.println("[CAM] Camera capture failed (fb null)");
    return nullptr;
  } else {
    Serial.printf("FB len=%u, first=%02X%02X, last=%02X%02X\n",
                  fb->len, fb->buf[0], fb->buf[1],
                  fb->buf[fb->len-2], fb->buf[fb->len-1]);
  }
  Serial.printf("[CAM] Captured image: len=%u, width=%u height=%u\n", fb->len, fb->width, fb->height);
  return fb;
}

#endif // CAM_UTILS_H