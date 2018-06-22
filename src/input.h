#ifndef ROBOT_INPUT
  #define ROBOT_INPUT

  #include <Arduino.h>
  #include <GoBLE.h>

  #define BLUETOOTH_SERIAL Serial1
  #define BLUETOOTH_STATE_PIN 2

  int readJoystickX();
  int readJoystickY();
  int readInput();
  void initInput();

#endif
