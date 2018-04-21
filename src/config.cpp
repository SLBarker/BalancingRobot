#include <Arduino.h>
#include "eepromStruct.h"
#include "config.h"

robotConfiguration createDefaultConfig() {
  robotConfiguration config;
  config.version = CONFIG_VERSION;

  config.motorConfig.enable = false;
  config.motorConfig.stepMode = thirtySecondStep;

  config.pidConfig.potential = 1.0;
  config.pidConfig.integral = 2.0;
  config.pidConfig.derivative = 3.0;
  
  return config;
}

robotConfiguration readConfig() {
    robotConfiguration config;
    int bytesRead = eepromRead (0,config);
    if (bytesRead == sizeof(config)) {
      if (config.version == CONFIG_VERSION) {
        return config;
      }
    }

    // something when wrong....
    config = createDefaultConfig();
    return config;
}

bool writeConfig(robotConfiguration config){
  int bytesWritten = eepromWrite(0, config);
  return bytesWritten == sizeof(config);
}
