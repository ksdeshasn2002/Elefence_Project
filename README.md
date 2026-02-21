# Elefence: AI-Powered Elephant Detection & Fence Monitoring

**Elefence** is an intelligent IoT prototype designed to mitigate human-elephant conflict. It utilizes on-device Computer Vision and motion sensing to detect elephant presence and monitor fence integrity in real-time.



---

### Project Disclaimer
This project is a **prototype** created as a learning exercise. It was developed with assistance from **Artificial Intelligence** (LLMs). While it demonstrates a complete end-to-end engineering flow, some functions may not work perfectly in all environmental conditions, and the system is intended for educational and demonstration purposes.

---

### Purpose
In regions like Sri Lanka, elephants frequently breach agricultural fences. This project aims to provide an automated system to:
* **Detect** elephants before or during a breach attempt.
* **Differentiate** between natural fence movement (wind/branches) and an actual elephant breach.
* **Alert** authorities or farmers immediately via digital notifications.

---

### Hardware Stack
* **Vision Node:** ESP32-CAM (AI-Thinker)
* **Main Processing Node:** ESP32-S3 (N16R8) — Chosen for its 16MB Flash and 8MB PSRAM.
* **Sensors:** PIR Motion Sensor and MPU6050 (Accelerometer/Gyroscope).
* **Communication:** High-speed UART (921600 baud) for inter-board data transfer.

---

### AI Model Specifications
The heart of the system is a custom-trained image classification model optimized for microcontrollers.
* **Framework:** TensorFlow Lite Micro.
* **Input Size:** 96x96 pixels (RGB).
* **Quantization:** Full Integer (INT8) internal quantization for speed, with Float32 Input/Output for compatibility.
* **Size:** ~2.8MB (Loaded into the ESP32-S3 PSRAM for high-speed access).

---

### Technical Pivot: From GSM to Web
Initially, the project was designed to communicate via **GSM (2G signals)** for remote areas. Due to budget constraints and hardware troubleshooting challenges, the architecture was pivoted to use **Web Servers**. The current version sends HTTP requests via WiFi to a central node server, which then handles the bulk alert notifications.

---

### Engineering Challenges & Solutions

**1. The "Ghost" Frame Buffer Overflow**
In the camera configurations, we encountered issues where `esp_camera_fb_return()` failed to properly clear the buffer, causing persistent overflows. 
* **Solution:** After extensive troubleshooting, we identified a compatibility issue with the latest ESP32 Arduino Board Manager. Downgrading the **Arduino ESP32 Board version** stabilized memory management and resolved the error.

**2. TFLite Memory & Operator Registration**
Loading a 2.8MB model required manual memory routing. 
* **Solution:** We manually allocated the **Tensor Arena (4MB)** and the model into the **8MB PSRAM**. We also manually registered specific operators (such as `MaxPool2D` and `Pad`) in the `MicroMutableOpResolver` to prevent crashes during inference.

**3. UART Synchronization**
At 921600 baud, the default serial buffer (256 bytes) couldn't handle the incoming image data.
* **Solution:** We expanded the RX buffer to **64KB** using `camSerial.setRxBufferSize(64 * 1024)` to ensure data integrity during transfer.

---

### System Logic
1. **Trigger:** The PIR sensor on the camera node detects motion.
2. **Transfer:** The camera captures a frame and sends it to the S3 node via UART.
3. **Inference:** The ESP32-S3 runs the image through the TFLite model.
4. **Drift Correction:** A custom `mpu_reset_timing()` function is used to prevent mathematical "drift" in the gyroscope while the processor is busy with AI inference.
5. **Detection Logic:**
    * **Elephant detected + Fence movement:** Triggers a "Breach Alert."
    * **No Elephant + Fence movement:** Triggers a "Natural Cause/Damage" notification.

---

### Installation
1. Upload the code in `ESP32_Cam_Node` to your ESP32-CAM.
2. Upload the code in `ESP32_S3_Main_Node` to your ESP32-S3.
3. **Note:** Set the Partition Scheme to **16MB Flash** in the Arduino IDE.
4. Connect Cam TX (14) to S3 RX (18) and Cam RX (15) to S3 TX (17). **Common Ground (GND) is required.**
