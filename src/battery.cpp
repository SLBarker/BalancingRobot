#include <Arduino.h>
#include <ADC.h>
#include "battery.h"

ADC *adc = new ADC();

void initBattery() {
  // configure the pin used for battery voltage level
  pinMode(BATTERY_LEVEL_PIN, INPUT);
  pinMode(BATTERY_STATUS_PIN, OUTPUT);

  adc->setAveraging(32); // set number of averages
  adc->setResolution(16); // set bits of resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED); // change the conversion speed
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED); // change the sampling speed
}

float readBatteryVoltage() {
  int reading = adc->adc0->analogRead(BATTERY_LEVEL_PIN);
  return reading*3.3*VOLTAGE_DIVIDER_RATIO/adc->getMaxValue(ADC_0) ;
  //return (MAX_PIN_VOLTAGE * VOLTAGE_DIVIDER_RATIO * reading) / MAX_READING;
}

// led will be off if voltage is above 11 volts
// below 11 volts the led will start to flash
// below 9.9 volts the led will stay on continuously.
boolean batteryOk() {
  static unsigned long nextCycle = 0;

  if (millis() < nextCycle)
    return true;

  int state = !digitalRead(BATTERY_STATUS_PIN);
  digitalWrite(BATTERY_STATUS_PIN, state);

  float voltage = readBatteryVoltage();
  Serial.printf("Battery: %fv\n", voltage);

  // how long should the led be on for...
  long cycleTime = max(0,min(BATTERY_MONITOR_RATE, (voltage - MIN_VOLTAGE) * FLASH_RATE_MULTIPLIER));

  if (state == HIGH)
    cycleTime = BATTERY_MONITOR_RATE - cycleTime;

  //Serial.printf("State: %s, Cycle length: %d\n", state==LOW?"off":"on", cycleTime);
  nextCycle = millis() + cycleTime;

  // return true if voltage is still ok.
  return voltage > MIN_VOLTAGE;
}
