#ifndef ROBOT_BATTERY
  #define ROBOT_BATTERY

  #define BATTERY_LEVEL_PIN A9
  // #define BATTERY_LEVEL_ANALOGUE_PIN 9
  #define MAX_VOLTAGE 3.3
  #define MAX_READING 1023
  #define VOLTAGE_DIVIDER_RATIO 4.65
  #include <Arduino.h>

  void initBattery();
  float readBatteryVoltage();
#endif
