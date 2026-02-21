#ifndef serial_UTILS_H
#define serial_UTILS_H

#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino.h>


#define CAM_UART Serial1   // or your chosen HardwareSerial
#define PACKET_HEADER_1 0xAA
#define PACKET_HEADER_2 0x55

bool sendImageViaSerial(uint8_t* data, size_t len) {

  if (!data || len == 0) {
    Serial.println("[UART] Invalid image buffer");
    return false;
  }

  Serial.printf("[UART] Sending heap image — %u bytes\n", len);

  // ----- header -----
  CAM_UART.write(PACKET_HEADER_1);
  CAM_UART.write(PACKET_HEADER_2);

  // ----- size (4 bytes little endian) -----
  uint32_t size = len;
  CAM_UART.write((uint8_t*)&size, 4);

  // ----- payload in chunks -----
  const size_t chunk = 1024;
  size_t sent = 0;

  while (sent < size) {
    size_t remaining = size - sent;
    size_t n = (remaining < chunk) ? remaining : chunk;
    CAM_UART.write(data + sent, n);
    CAM_UART.flush();
    sent += n;
  }

  Serial.println("[UART] Image send complete");
  return true;
}

#endif // HTTP_UTILS_H