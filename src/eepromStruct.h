#ifndef ROBOT_EEPROM
  #define ROBOT_EEPROM

  #include <EEPROM.h>
  #include <Arduino.h>

  template <class T> int eepromWrite(int ee, const T& value)
  {
      const byte* p = (const byte*)(const void*)&value;
      unsigned int i;
      for (i = 0; i < sizeof(value); i++)
            EEPROM.write(ee++, *p++);
      return i;
  }

  template <class T> int eepromRead(int ee, T& value)
  {
      byte* p = (byte*)(void*)&value;
      unsigned int i;
      for (i = 0; i < sizeof(value); i++)
            *p++ = EEPROM.read(ee++);
      return i;
  }
#endif
