#ifndef PIR_UTILS_H
#define PIR_UTILS_H

#include <Arduino.h>

#define PIR_PIN 13

void pir_setup() {
  pinMode(PIR_PIN, INPUT);
}

bool pir_motion() {
  return digitalRead(PIR_PIN) == HIGH;
}

#endif // PIR_UTILS_H
