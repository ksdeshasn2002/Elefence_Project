#include <WiFi.h>
#include "model_data.h"
HardwareSerial camSerial(1);
#include "comunication_utils.h"
#include "mpu_utils.h"
#define UART_TX 17
#define UART_RX 18


const char* T_URL = "twilo_url";




// PIR + MPU monitoring state
bool elephantDetected = false;
unsigned long monitorStartTime = 0;

// Breach tracking variables to prevent spam
bool breachInformed = false;           // Elephant breach API called
bool naturalCauseInformed = false;     // Natural cause API called
unsigned long lastBreachTime = 0;      // When elephant breach was detected
unsigned long lastNaturalCauseTime = 0; // When natural cause was detected

// Time intervals
const unsigned long BREACH_RESET_TIME = 3600000UL;      // 1 hour (3600000ms)
const unsigned long NATURAL_CAUSE_INTERVAL = 3600000UL;  // 1 hour between natural cause alerts
const unsigned long GRACE_PERIOD_MS = 300000UL;         // 5 minutes grace period after elephant breach


#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include "esp_heap_caps.h"

#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASS "WIFI_PASS"


const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
tflite::MicroMutableOpResolver<15> resolver;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;
uint8_t* model_tflite_psram = nullptr;

constexpr size_t kTensorArenaSize = 4 * 1024 * 1024;
uint8_t* tensor_arena = nullptr;

String runInference(uint8_t* img_buf, size_t len) {
  Serial.printf("[AI] Starting inference with image size: %zu bytes\n", len);

  if (len < 4 || img_buf[0] != 0xFF || img_buf[1] != 0xD8) {
  return "ERR:BAD_JPEG";
}
  
  camera_fb_t fb;
  fb.buf = img_buf;
  fb.len = len;
  fb.width = 320;
  fb.height = 240;
  fb.format = PIXFORMAT_JPEG;

  Serial.println("[AI] Starting image preprocessing...");

  static uint8_t rgb_resized[96 * 96 * 3];

if (!decodeResizeToRGB(&fb, rgb_resized, 96, 96)) {
  return "ERR:DECODE";
}

preprocessImageToTensor(
    rgb_resized,
    input->data.f,
    96,
    96
);

  Serial.println("[AI] Preprocessing completed");

  Serial.println("[AI] Running inference...");

  if (interpreter->Invoke() != kTfLiteOk) {
    return "ERR:INVOKE";
  }
  Serial.println("[AI] Inference completed");
  float score;

    if (output->type == kTfLiteUInt8) {
      float scale = output->params.scale;
      int zp = output->params.zero_point;
      score = scale * (output->data.uint8[0] - zp);
    }
    else if (output->type == kTfLiteFloat32) {
      score = output->data.f[0];
    }

    if (score > 0.5) {
      return "Elephant";
    } else {
      return "No Elephant";
    }


  Serial.printf("[AI] Scores Elephant: %d\n", score);


}


void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);
    Serial.println("\n[MAIN] Serial AI node boot");
    
  camSerial.setRxBufferSize(64 * 1024);


  camSerial.begin(SERIAL_BAUD, SERIAL_8N1, UART_RX, UART_TX);
  Serial.println("[UART] CAM serial ready");

    // init the mpu unit
  if (!mpu_setup()) {
    Serial.println("[ERROR] MPU init failed, continuing without MPU...");
  }
  connectWiFi(WIFI_SSID,WIFI_PASS);
  
  Serial.printf("Total heap: %d bytes\n", ESP.getHeapSize());
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Total PSRAM: %d bytes\n", ESP.getPsramSize());
  Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());



  Serial.println("[SETUP] Loading AI model...");
  model_tflite_psram = (uint8_t*)heap_caps_malloc(model_tflite_len, MALLOC_CAP_SPIRAM);
  if (!model_tflite_psram) {
    Serial.println("[ERROR] Failed to allocate model in PSRAM");
    while (1) delay(1000);
  }
  memcpy_P(model_tflite_psram, model_tflite, model_tflite_len);
  Serial.printf("[SETUP] Model loaded (%d bytes)\n", model_tflite_len);

  Serial.println("[SETUP] Initializing TensorFlow...");
  model = tflite::GetModel(model_tflite_psram);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("[ERROR] Model version mismatch");
    while (1) delay(1000);
  }

  tensor_arena = (uint8_t*)heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM);
  if (!tensor_arena) {
    Serial.println("[ERROR] Failed to allocate tensor arena");
    while (1) delay(1000);
  }
  // REGISTER OPERATORS 
  // 1. Convolution Layers
  resolver.AddConv2D();
  resolver.AddDepthwiseConv2D();
  
  // 2. Pooling & Reshaping 
  resolver.AddMaxPool2D(); 
  resolver.AddAveragePool2D();
  resolver.AddReshape();
  resolver.AddPad();  
  
  // 3. Mathematical & Activation
  resolver.AddAdd();
  resolver.AddMean();
  resolver.AddLogistic();   // (Sigmoid)
  resolver.AddSoftmax();
  resolver.AddFullyConnected();
  resolver.AddMul();        // <--- Added just in case
  
  // 4. Quantization
  resolver.AddQuantize();
  resolver.AddDequantize(); 

  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  Serial.println("[SETUP] Allocating Tensors..."); // Debug print
  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("[ERROR] Tensor allocation failed!");
    while (1) delay(1000);
  }
  Serial.println("[SETUP] Tensors Allocated Successfully!");

  input = interpreter->input(0);
  output = interpreter->output(0);

}

void loop() {

  // wait packet header
  if (!waitForHeader()) {
    Serial.println("waiting for headers");
    delay(10);
    return;
  }

  Serial.println("\n[UART] Header received! Reading data...");

  // read image size (4 bytes little endian)
  uint32_t imgSize;
  if (!readExact((uint8_t*)&imgSize, 4)) {
      Serial.println("[ERR] Failed to read image size");
      return;
  }
  Serial.printf("[UART] Expecting image size: %d bytes\n", imgSize);

  if (imgSize == 0 || imgSize > MAX_IMAGE_SIZE) {
    Serial.println("ERR:SIZE");
    return;
  }

  uint8_t* imgBuf = (uint8_t*)heap_caps_malloc(imgSize, MALLOC_CAP_SPIRAM);
  if (!imgBuf) {
    Serial.println("ERR:MEM");
    return;
  }

  if (!readExact(imgBuf, imgSize)) {
    free(imgBuf);
    Serial.println("ERR:READ");
    return;
  }

  String result = runInference(imgBuf, imgSize);
  Serial.println(result);
  free(imgBuf);
  if(result=="Elephant"){
    elephantDetected = true;
  }
  
  mpu_reset_timing();


    // ===== IMPROVED ELEPHANT MONITORING WITH BREACH TRACKING =====
  
  if (elephantDetected) {
    Serial.println("[MONITOR] Starting 60-second fence monitoring...");
    monitorStartTime = millis();
    
    while (millis() - monitorStartTime <= 60000) {  // 1 minute window
      if (angle_exceeded()) {
        if (!breachInformed) {
          Serial.println("[ALERT] Fence angle exceeded during Elephant detection! Triggering Bulk Call...");

           triggerBulkCall(T_URL);
          
          breachInformed = true;
          lastBreachTime = millis();
          Serial.println("[ALERT] Elephant breach API called. Breach tracking activated.");
        } else {
          Serial.println("[INFO] Fence still breached but API already called.");
        }
        elephantDetected = false;  // Exit elephant monitoring
        Serial.println("[MONITOR] Elephant monitoring stopped due to fence breach.");
        break;
      }
      delay(500);  // Check every 500ms during monitoring
    }
    
    // If we exit the loop without triggering alert, reset elephant detection
    if (elephantDetected) {
      Serial.println("[MONITOR] 60-second monitoring complete - no fence breach detected.");
      elephantDetected = false;
    }
  }

  // ===== HANDLE FENCE MOVEMENT WITHOUT ELEPHANT (NATURAL CAUSES) =====
  if (!elephantDetected && angle_exceeded()) {
    // CRITICAL: 5-minute grace period after any elephant breach
    if (lastBreachTime > 0 && (millis() - lastBreachTime < GRACE_PERIOD_MS)) {
      unsigned long graceRemaining = (GRACE_PERIOD_MS - (millis() - lastBreachTime)) / 60000;
      Serial.println("[MPU] Fence angle exceeded but within 5-minute grace period after elephant breach. Grace time remaining: " + 
                    String(graceRemaining) + " minutes");
      // Skip rest of main loop to avoid spam
      delay(200);
      return;
    }
    
    // Only consider natural causes if enough time has passed since elephant breach
    if (!breachInformed) {
      
      // Check if we should inform about natural cause
      if (!naturalCauseInformed || (millis() - lastNaturalCauseTime > NATURAL_CAUSE_INTERVAL)) {
        Serial.println("[MPU] Fence moved without elephant detection -> Natural cause suspected.");
        Serial.println("[ALERT] Calling Natural Cause API...");
        String massage = "there is a damage in the fence near the pole number 6";
         sendNotification(T_URL,massage);
        
        naturalCauseInformed = true;
        lastNaturalCauseTime = millis();
        Serial.println("[INFO] Natural cause API called. Next alert in 1 hour.");
      } else {
        unsigned long remainingMinutes = (NATURAL_CAUSE_INTERVAL - (millis() - lastNaturalCauseTime)) / 60000;
        Serial.println("[MPU] Natural cause detected but alert already sent. Next alert in: " + 
                      String(remainingMinutes) + " minutes");
      }
    } else {
      Serial.println("[MPU] Fence angle exceeded but recent elephant breach detected. Waiting for 1-hour reset.");
    }
  }

  // ===== RESET TRACKING VARIABLES AFTER TIME PERIODS =====
  if (breachInformed && (millis() - lastBreachTime > BREACH_RESET_TIME)) {
    Serial.println("[RESET] Breach tracking reset after 1 hour.");
    breachInformed = false;
    naturalCauseInformed = false;  // Also reset natural cause when breach resets
  }

delay(200);

}