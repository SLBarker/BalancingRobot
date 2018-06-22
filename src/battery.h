#ifndef ROBOT_BATTERY
  #define ROBOT_BATTERY

  #define BATTERY_LEVEL_PIN A14
  #define BATTERY_STATUS_PIN 13

  #define MAX_PIN_VOLTAGE 3.3
  #define MAX_READING 1023
  #define VOLTAGE_DIVIDER_RATIO 4.3


  // constants for battery led monitor.
  #define BATTERY_MONITOR_RATE 5000
  #define MAX_VOLTAGE 11.0
  #define MIN_VOLTAGE 9.9
  #define FLASH_RATE_MULTIPLIER BATTERY_MONITOR_RATE / (MAX_VOLTAGE - MIN_VOLTAGE)

  #include <Arduino.h>

  void initBattery();
  bool batteryOk();
  float readBatteryVoltage();
#endif
