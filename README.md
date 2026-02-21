Elefence: AI-Powered Elephant Detection & Fence Monitoring
Elefence is an intelligent IoT prototype designed to mitigate human-elephant conflict. It uses on-device Computer Vision and motion sensing to detect elephant presence and monitor fence integrity in real-time.

⚠️ Project Disclaimer
This project is a prototype created as a learning exercise. It was developed with significant assistance from Artificial Intelligence (LLMs). While it demonstrates a complete end-to-end engineering flow, some functions may not work perfectly in all environmental conditions, and the system is intended for educational and demonstration purposes.

🎯 Purpose
In regions like Sri Lanka, elephants frequently breach agricultural fences. This project aims to provide an automated way to:

Detect elephants before or during a breach.

Differentiate between natural fence movement (wind/branches) and an actual elephant breach.

Alert authorities or farmers immediately via digital notifications.

🛠️ Hardware Stack
Vision Node: ESP32-CAM (AI-Thinker)

Main Processing Node: ESP32-S3 (N16R8) — Selected for its 16MB Flash and 8MB PSRAM.

Sensors: PIR Motion Sensor, MPU6050 Accelerometer/Gyroscope.

Communication: High-speed UART (921600 baud) for inter-board data transfer.

🧠 AI Model Specifications
Framework: TensorFlow Lite Micro.

Input Size: 96x96 pixels (RGB).

Quantization: Full Integer (INT8) internal quantization with Float32 Input/Output for compatibility.

Size: ~2.8MB (Allocated in PSRAM using MALLOC_CAP_SPIRAM).

📡 Technical Pivot: From GSM to Web
Initially, we planned to use GSM (2G) for remote connectivity. However, due to budget constraints and hardware troubleshooting issues, we pivoted to a Web Server architecture. The system currently sends HTTP requests via WiFi to a central server to handle notifications.

🚀 Key Engineering Challenges & Solutions
1. The "Ghost" Frame Buffer Overflow
In the camera_utils.h configurations, we encountered persistent memory issues where esp_camera_fb_return() failed to properly clear the buffer, leading to immediate overflows. Even after optimizing the code, the error remained.

Solution: We discovered a compatibility issue with the latest ESP32 Arduino Board Manager. By downgrading the Arduino ESP32 Board version, the memory management stabilized and the overflow errors disappeared.

2. TFLite Memory & Operators
Loading a 2.8MB model on a microcontroller required manual memory routing.

Solution: We manually allocated the Tensor Arena (4MB) and the model itself into the 8MB PSRAM. We also had to manually register missing operators (like MaxPool2D and Pad) in the MicroMutableOpResolver to prevent silent crashes during inference.

3. High-Speed UART Synchronization
At 921600 baud, the Main Node's serial buffer (default 256 bytes) couldn't keep up with the image data "firehose."

Solution: We expanded the RX buffer to 64KB using camSerial.setRxBufferSize(64 * 1024) before initialization to ensure zero packet loss.

⚙️ How It Works
Trigger: PIR sensor wakes the camera node.

Transfer: Image sent via Serial1 to the S3 node.

Inference: ESP32-S3 runs the TFLite model.

Logic: * Elephant detected + Fence move: Elephant Breach Alert.

No Elephant + Fence move: Natural Cause/Maintenance Alert.

Drift Correction: Used mpu_reset_timing() to prevent the gyroscope math from "exploding" during the long AI processing time.
