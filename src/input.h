#ifndef ROBOT_INPUT
  #define ROBOT_INPUT

  #include <Arduino.h>
  #include <GoBLE.h>

  #define BLUETOOTH_SERIAL Serial5

  int readJoystickX();
  int readJoystickY();
  int readInput();

#endif
