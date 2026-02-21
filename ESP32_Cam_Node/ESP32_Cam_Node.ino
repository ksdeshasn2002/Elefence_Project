#include <Arduino.h>
#include "camera_utils.h"
#include "serial_utils.h"
#include "pir_utils.h"

#define UART_TX 14
#define UART_RX 15

// Rate limits
const unsigned long CAPTURE_COOLDOWN_MS = 10000UL; // Increased cooldown to 10 seconds
const unsigned long PIR_DEBOUNCE_MS = 2000UL; // PIR debounce time
unsigned long lastCaptureTime = 0;
unsigned long lastPirTime = 0;


void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("[CAM] booting...");
  
  Serial1.begin(921600, SERIAL_8N1, UART_RX, UART_TX);

  
  pir_setup();
  Serial.println(" Waiting for PIR...");

  // Initialize camera once at startup
  if (!camera_init()) {
    Serial.println("[ERROR] Camera init failed, rebooting...");
    ESP.restart();
  }
       
  // Clear any initial frames to stabilize the camera
  for (int i = 0; i < 3; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      esp_camera_fb_return(fb);
    }
    delay(100);
  } 
  Serial.println("[CAM] Camera initialized and stabilized, ready.");
  
  Serial.println("[BOOT] All systems ready.");
}

void loop() {
  unsigned long now = millis();
  
  if (pir_motion()) {
    // PIR debouncing
    if (now - lastPirTime < PIR_DEBOUNCE_MS) {
      delay(100);
      return;
    }
    lastPirTime = now;
    
    Serial.println("PIR detected");
    
    if (now - lastCaptureTime >= CAPTURE_COOLDOWN_MS) {
      Serial.println("[CAM] PIR triggered - capturing image...");
      
      // Clear any stale frames first
      for (int i = 0; i < 2; i++) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) {
          esp_camera_fb_return(fb);
        }
        delay(50);
      }
      
      // Capture the actual frame
      camera_fb_t* fb = esp_camera_fb_get();
      
      if (fb) {
        Serial.printf("FB len=%u, fmt=%d\n", fb->len, fb->format);
        
        // Check if buffer is reasonable size
        if (fb->len == 0 || fb->len > 100000) {
          Serial.printf("[CAM] Invalid buffer size: %u\n", fb->len);
          esp_camera_fb_return(fb);
          return;
        }
        
        // Create a heap copy to avoid buffer issues during HTTP transmission
        uint8_t* heapCopy = (uint8_t*) heap_caps_malloc(fb->len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!heapCopy) {
          // Fallback to regular malloc if PSRAM allocation fails
          heapCopy = (uint8_t*) malloc(fb->len);
        }
        
        if (heapCopy) {
          memcpy(heapCopy, fb->buf, fb->len);
          size_t imageLen = fb->len;
          
          // Return the frame buffer immediately to prevent overflow
          esp_camera_fb_return(fb);
          fb = nullptr;
          
          Serial.println("[CAM] Captured frame, sending...");
          
          // CRITICAL: Reset MPU timing before HTTP request to prevent drift

          
          if(sendImageViaSerial(heapCopy,imageLen)){
            Serial.println("[ERROR] Heap sending succussfull");
         

          }else{Serial.println("[ERROR] Heap sending failed");
          return;
          }

          // Free the heap copy
          free(heapCopy);
        } else {
          Serial.println("[ERROR] Heap allocation failed!");
          esp_camera_fb_return(fb);
        }
      } else {
        Serial.println("[CAM] Capture failed.");
      }
      
      lastCaptureTime = now;
    } else {
      Serial.println("[CAM] PIR detected but in cooldown.");
    }
  } else {
    Serial.println("NO MOTION DETECTED");
  }

  delay(200); // Main loop delay
}