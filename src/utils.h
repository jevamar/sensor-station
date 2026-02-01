/*
* Утилиты для управления переферией контроллера
*/
#pragma once

#include "Esp.h"

#ifdef ARDUINO_ARCH_ESP8266
  #include <ESP8266WiFi.h>
  #define CHIP_ID ESP.getChipId()
#elif defined(ARDUINO_ARCH_ESP32)
  #include <WiFi.h>
  #include <esp_system.h>
  #define CHIP_ID ((uint32_t)(ESP.getEfuseMac() >> 32))
#endif

void ledInit(uint8_t pin);
void ledOn(uint8_t pin);
void ledOff(uint8_t pin);

inline void printChipInfo() {
  Serial.print("Chip ID: ");
  Serial.println(CHIP_ID);
  
  #ifdef ESP8266
    Serial.print("Flash Size: ");
    Serial.print(ESP.getFlashChipSize() / 1024 / 1024);
    Serial.println(" MB");
  #elif defined(ESP32)
    Serial.print("Free Heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");
  #endif
}