#include <Arduino.h>
#include "input.h"

_GoBLE goble(&BLUETOOTH_SERIAL);

int readInput() {
  int ch = -1;
  if (goble.available()) {

    if (goble.readSwitchUp() == PRESSED)
      ch = '+';

    if (goble.readSwitchDown() == PRESSED)
      ch = '-';

    if (goble.readSwitchLeft() == PRESSED)
      ch = '<';

    if (goble.readSwitchRight() == PRESSED)
      ch = '>';

    if (goble.readSwitchSelect() == PRESSED)
      ch = '/';

    if (goble.readSwitchStart() == PRESSED)
      ch = '*';
  }
  return ch;
}

int readJoystickX() {
  return goble.readJoystickX()-128;
}

int readJoystickY() {
  return goble.readJoystickY()-128;
}
