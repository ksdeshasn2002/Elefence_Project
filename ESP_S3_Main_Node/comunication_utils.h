#ifndef COMUNICATION_UTILS_H 
#define COMUNICATION_UTILS_H 
#include <Arduino.h>
#include "esp_camera.h"
#include "img_converters.h"
#include <WiFi.h>
#include <base64.h> 
#include "fb_gfx.h"
#include <HTTPClient.h>


#define SERIAL_BAUD 921600
#define PACKET_HEADER_1 0xAA
#define PACKET_HEADER_2 0x55
#define MAX_IMAGE_SIZE 120000
// Model input size
#define MODEL_INPUT_WIDTH 96
#define MODEL_INPUT_HEIGHT 96
#define MODEL_INPUT_CHANNELS 3  // trained on RGB

extern HardwareSerial camSerial;


void connectWiFi(const char* ssid, const char* pass) {
  Serial.printf("[WIFI] Connecting to %s\n", ssid);
  WiFi.begin(ssid, pass);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > 20000) {
      Serial.println("[WIFI] Timeout connecting to WiFi");
      break;
    }
    delay(300);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("[WIFI] Connected. IP: " + WiFi.localIP().toString());
  } else {
    Serial.println();
    Serial.println("[WIFI] Not connected.");
  }
}



bool waitForHeader() {
  while (camSerial.available() >= 2) {
    if (camSerial.read() == PACKET_HEADER_1 &&
        camSerial.read() == PACKET_HEADER_2) {
      return true;
    }
  }
  return false;
}



bool readExact(uint8_t* buf, size_t len) {
  size_t got = 0;
  unsigned long timeout = millis() + 10000;

  while (got < len && millis() < timeout) {
    if (camSerial.available()) {       
      got += camSerial.readBytes(buf + got, len - got); 
    }
  }
  return got == len;
}

bool receiveImage(uint8_t** outBuf, size_t* outLen){
  // wait start marker
  if (camSerial.read() != 0xAA) return false;
  if (camSerial.read() != 0x55) return false;

  // read size
  uint32_t len;
  camSerial.readBytes((char*)&len, 4);

  uint8_t* buf = (uint8_t*) heap_caps_malloc(len, MALLOC_CAP_SPIRAM);
  if (!buf) return false;

  size_t read = camSerial.readBytes(buf, len);
  if (read != len) {
    free(buf);
    return false;
  }

  if (camSerial.read() != 0x55) { free(buf); return false; }
  if (camSerial.read() != 0xAA) { free(buf); return false; }

  *outBuf = buf;
  *outLen = len;
  return true;
}

bool decodeResizeToRGB(
    camera_fb_t* fb,
    uint8_t* out_rgb,
    int target_w,
    int target_h
) {
  if (!fb || fb->format != PIXFORMAT_JPEG) {
    Serial.println("[IMG] Not JPEG format");
    return false;
  }

  // allocate temp full RGB buffer
  uint8_t* rgb888 = (uint8_t*) heap_caps_malloc(
      fb->width * fb->height * 3,
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );

  if (!rgb888) {
    Serial.println("[IMG] RGB buffer alloc failed");
    return false;
  }

  // decode JPEG → RGB888
  bool ok = fmt2rgb888(fb->buf, fb->len, fb->format, rgb888);
  if (!ok) {
    Serial.println("[IMG] JPEG decode failed");
    free(rgb888);
    return false;
  }

  // nearest-neighbor resize
  for (int y = 0; y < target_h; y++) {
    int src_y = y * fb->height / target_h;

    for (int x = 0; x < target_w; x++) {
      int src_x = x * fb->width / target_w;

      int src_index = (src_y * fb->width + src_x) * 3;
      int dst_index = (y * target_w + x) * 3;

      out_rgb[dst_index + 0] = rgb888[src_index + 0];
      out_rgb[dst_index + 1] = rgb888[src_index + 1];
      out_rgb[dst_index + 2] = rgb888[src_index + 2];
    }
  }

  free(rgb888);
  return true;
}

void preprocessImageToTensor(
    uint8_t* rgb,
    float* out,
    int width,
    int height
) {
  const float contrast = 1.3f;

  int in_i = 0;
  int out_i = 0;

  for (int i = 0; i < width * height; i++) {

    float R = rgb[in_i++];
    float G = rgb[in_i++];
    float B = rgb[in_i++];

    // grayscale
    float gray = 0.299f * R + 0.587f * G + 0.114f * B;

    // contrast
    gray = (gray - 128.0f) * contrast + 128.0f;

    if (gray < 0) gray = 0;
    if (gray > 255) gray = 255;

    // MobileNet preprocess_input
    float norm = (gray / 127.5f) - 1.0f;

    // duplicate to RGB
    out[out_i++] = norm;
    out[out_i++] = norm;
    out[out_i++] = norm;
  }
}


void sendResultToCam(const String& msg) {
  uint8_t len = msg.length();

  camSerial.write(0xCC);
  camSerial.write(0x33);
  camSerial.write(len);
  camSerial.write((const uint8_t*)msg.c_str(), len);

  camSerial.flush();

  Serial.println("[UART] Result sent to CAM: " + msg);
}

void triggerBulkCall(String serverUrl) {
  Serial.println("[API] Triggering emergency bulk call...");
  
  HTTPClient http;
  http.setTimeout(15000); // 15 second timeout for call API
  http.setConnectTimeout(5000); // 5 second connect timeout
  
  String fullUrl = serverUrl + "/call";
  Serial.printf("[API] Calling: %s\n", fullUrl.c_str());
  
  http.begin(fullUrl);
  http.addHeader("Content-Type", "application/json");
  
  // Your server doesn't need a body for /call endpoint - it uses hardcoded numbers
  String payload = "{}"; // Empty JSON object
  
  int httpCode = http.POST(payload);
  
  if (httpCode > 0) {
    Serial.printf("[API] Call API Response code: %d\n", httpCode);
    String response = http.getString();
    Serial.printf("[API] Call API Response: %s\n", response.c_str());
    
    if (httpCode >= 200 && httpCode < 300) {
      Serial.println("[API] ✓ Emergency bulk call initiated successfully!");
    } else {
      Serial.printf("[API] ✗ Call API failed with code: %d\n", httpCode);
    }
  } else {
    Serial.printf("[API] ✗ Call API request failed: %d\n", httpCode);
  }
  
  http.end();
}

void sendNotification(String serverUrl, String message) {
  Serial.printf("[NOTIFY] Sending SMS notification: %s\n", message.c_str());
  
  HTTPClient http;
  http.setTimeout(10000); // 10 second timeout for SMS
  http.setConnectTimeout(5000);
  
  String fullUrl = serverUrl + "/notify";
  Serial.printf("[NOTIFY] SMS URL: %s\n", fullUrl.c_str());
  
  http.begin(fullUrl);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("User-Agent", "ESP32-ElephantFence");
  
  // Create notification payload for your Twilio server
  // Your server uses AUTHORITY_NUMBER from hardcoded value, so we don't need "to"
  String payload = "{";
  payload += "\"message\":\"" + message + "\"";
  payload += "}";
  
  Serial.printf("[NOTIFY] SMS Payload: %s\n", payload.c_str());
  
  int httpCode = http.POST(payload);
  
  if (httpCode > 0) {
    Serial.printf("[NOTIFY] SMS Response code: %d\n", httpCode);
    String response = http.getString();
    Serial.printf("[NOTIFY] SMS Response: %s\n", response.c_str());
    
    if (httpCode >= 200 && httpCode < 300) {
      Serial.println("[NOTIFY] ✓ SMS notification sent successfully!");
    } else {
      Serial.printf("[NOTIFY] ✗ SMS failed with code: %d\n", httpCode);
    }
  } else {
    Serial.printf("[NOTIFY] ✗ SMS request failed: %d\n", httpCode);
  }
  
  http.end();
}



#endif


