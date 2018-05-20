#include <Arduino.h>
#include "battery.h"

void initBattery() {
  // configure the pin used for battery voltage level
  pinMode(BATTERY_LEVEL_PIN, INPUT);
}

float readBatteryVoltage() {
  int reading = analogRead(BATTERY_LEVEL_PIN);
  return (MAX_VOLTAGE * VOLTAGE_DIVIDER_RATIO * reading) / MAX_READING;
}
