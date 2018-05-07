#include <Arduino.h>
#include "eepromStruct.h"
#include "motor.h"
#include "config.h"
#include "mpu.h"


robotConfiguration createDefaultConfig() {
  robotConfiguration config;
  config.version = CONFIG_VERSION;

  config.motorConfig.enabled = false;
  config.motorConfig.stepMode = MOTOR_THIRTYSECOND_STEP;

  config.pidConfig.potential = 1.0;
  config.pidConfig.integral = 2.0;
  config.pidConfig.derivative = 3.0;

  config.mpuConfig.xGyroOffset = 0;
  config.mpuConfig.yGyroOffset = 0;
  config.mpuConfig.zGyroOffset = 0;

  config.mpuConfig.xAccelOffset = 0;
  config.mpuConfig.yAccelOffset = 0;
  config.mpuConfig.zAccelOffset = 2160;

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

void applyMotorConfig(motorConfiguration motorConfig) {
  enableMotors(motorConfig.enabled);
  setMotorStep(motorConfig.stepMode);
}

void applyMpuConfig(mpuCalibrationConfiguration mpuConfig) {
  setMpuOffsets(mpuConfig);
}

void applyConfig(robotConfiguration config) {
  applyMotorConfig(config.motorConfig);
  applyMpuConfig(config.mpuConfig);
}
