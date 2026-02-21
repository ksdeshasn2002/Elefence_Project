#ifndef MPU_UTILS_H
#define MPU_UTILS_H

#include <Wire.h>
#include <MPU6050.h>

#define SDA_PIN 8
#define SCL_PIN 9

MPU6050 mpu;
unsigned long lastReadTime = 0;

// Threshold for angle change
const float ANGLE_THRESHOLD = 50.0; // degrees (tune as needed)

// Kalman filter / complementary filter vars
float accAngleX, accAngleY;
float gyroAngleX = 0, gyroAngleY = 0;
float roll = 0, pitch = 0;
float lastTime = 0;

// Timing protection
const float MAX_DT = 0.1; // Maximum allowed dt (100ms)
bool mpuInitialized = false;

// Debug output control
unsigned long lastDebugTime = 0;
const unsigned long DEBUG_INTERVAL = 2000; // Print debug info every 2 seconds

// Initialization
bool mpu_setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("[MPU] ERROR: MPU6050 connection failed!");
    return false;
  }
  
  Serial.println("[MPU] MPU6050 ready.");
  
  // Initialize timing and angles
  lastTime = millis();
  gyroAngleX = 0;
  gyroAngleY = 0;
  roll = 0;
  pitch = 0;
  mpuInitialized = true;
  
  // Calibration: Take initial accelerometer reading as baseline
  delay(100);
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Set initial angles from accelerometer
  roll = atan2(ay, az) * 180 / PI;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;
  
  Serial.printf("[MPU] Initial calibration - Roll: %.2f°, Pitch: %.2f°\n", roll, pitch);
  
  return true;
}

// Reset MPU timing (call before long operations like HTTP requests)
void mpu_reset_timing() {
  lastTime = millis();
  Serial.println("[MPU] Timing reset - preventing drift during long operations");
}

// Read and calculate angle with timing protection
float getFenceAngle() {
  if (!mpuInitialized) {
    Serial.println("[MPU] ERROR: MPU not initialized!");
    return 0.0;
  }
  
  // Get raw values
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate accelerometer angle
  accAngleX = atan2(ay, az) * 180 / PI;
  accAngleY = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

  // Calculate time difference with protection
  float currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  
  // CRITICAL: Protect against large dt values (from HTTP delays, etc.)
  if (dt > MAX_DT) {
    Serial.printf("[MPU] WARNING: Large dt detected (%.3fs), capping to %.3fs\n", dt, MAX_DT);
    dt = MAX_DT;
    
    // Reset gyro integration to prevent massive drift
    gyroAngleX = 0;
    gyroAngleY = 0;
    
    // Use accelerometer values as reference
    roll = accAngleX;
    pitch = accAngleY;
    
    lastTime = currentTime;
    return pitch;
  }
  
  lastTime = currentTime;
  
  // Only integrate gyro if dt is reasonable
  if (dt > 0.001) { // Minimum 1ms
    gyroAngleX += gx / 131.0 * dt;
    gyroAngleY += gy / 131.0 * dt;

    // Complementary filter with drift protection
    roll  = 0.98 * (roll + gx / 131.0 * dt) + 0.02 * accAngleX;
    pitch = 0.98 * (pitch + gy / 131.0 * dt) + 0.02 * accAngleY;
    
    // Prevent extreme values (sensor malfunction protection)
    if (abs(roll) > 180) {
      roll = accAngleX; // Reset to accelerometer value
      Serial.println("[MPU] Roll reset due to extreme value");
    }
    if (abs(pitch) > 180) {
      pitch = accAngleY; // Reset to accelerometer value
      Serial.println("[MPU] Pitch reset due to extreme value");
    }
  }

  // Debug output with reduced frequency
  unsigned long now = millis();
  if (now - lastDebugTime > DEBUG_INTERVAL) {
    Serial.printf("[MPU] Roll: %.2f°, Pitch: %.2f°, dt: %.3fs\n", roll, pitch, dt);
    lastDebugTime = now;
  }

  return pitch; // Return pitch for fence orientation
}

// Check threshold with controlled debug output
bool angle_exceeded() {
  float angle = getFenceAngle();
  
  // Only print when threshold is actually exceeded
  bool exceeded = fabs(angle) > ANGLE_THRESHOLD;
  if (exceeded) {
    Serial.printf("[MPU] *** THRESHOLD EXCEEDED! *** Angle=%.2f° (threshold: %.1f°)\n", 
                  angle, ANGLE_THRESHOLD);
  }
  
  return exceeded;
}

// Function to get current angle without debug output (for monitoring loops)
float getCurrentAngle() {
  if (!mpuInitialized) return 0.0;
  
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  
  // Same timing protection
  if (dt > MAX_DT) {
    dt = MAX_DT;
    gyroAngleX = 0;
    gyroAngleY = 0;
    roll = atan2(ay, az) * 180 / PI;
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;
    lastTime = currentTime;
    return pitch;
  }
  
  if (dt > 0.001) {
    accAngleX = atan2(ay, az) * 180 / PI;
    accAngleY = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;
    
    roll = 0.98 * (roll + gx / 131.0 * dt) + 0.02 * accAngleX;
    pitch = 0.98 * (pitch + gy / 131.0 * dt) + 0.02 * accAngleY;
    
    // Extreme value protection
    if (abs(roll) > 180) roll = accAngleX;
    if (abs(pitch) > 180) pitch = accAngleY;
  }
  
  lastTime = currentTime;
  return pitch;
}

// Silent threshold check (for use in monitoring loops)
bool angle_exceeded_silent() {
  float angle = getCurrentAngle();
  return fabs(angle) > ANGLE_THRESHOLD;
}

#endif