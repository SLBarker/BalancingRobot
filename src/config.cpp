#include <Arduino.h>
#include <stepperMgr.h>
#include <stepper.h>
#include "eepromStruct.h"
//#include "motor.h"
#include "config.h"
#include "mpu.h"

robotConfiguration robotConfig;

robotConfiguration createDefaultConfig() {
  robotConfiguration config;
  config.version = CONFIG_VERSION;

  config.motorConfig.enabled = false;
  config.motorConfig.stepMode = MOTOR_THIRTYSECOND_STEP;

  config.pidConfig.kp = 0;
  config.pidConfig.ki = 0;
  config.pidConfig.kd = 0;

  config.mpuConfig.xGyroOffset = 0;
  config.mpuConfig.yGyroOffset = 0;
  config.mpuConfig.zGyroOffset = 0;

  config.mpuConfig.xAccelOffset = 0;
  config.mpuConfig.yAccelOffset = 0;
  config.mpuConfig.zAccelOffset = 2160;

  config.pidAutoTuneConfig.outputStep = 0.3;  //Turn motor at 0.3 rps
  config.pidAutoTuneConfig.controlType = 1; // 1= PID 0 = PI
  config.pidAutoTuneConfig.lookbackSec = 1; // half second - assume oscilation of about 2-3 seconds.
  config.pidAutoTuneConfig.noiseBand = 0.05; // input value;

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
  Serial.printf("Motor Enabled:%s\n", motorConfig.enabled?"true":"false");
  mgr.enableMotors(motorConfig.enabled);
  Serial.printf("Motor Step:%d\n", motorConfig.stepMode);
  mgr.setStepMode(motorConfig.stepMode);
}

void applyMpuConfig(mpuCalibrationConfiguration mpuConfig) {
  setMpuOffsets(mpuConfig);
}

void applyPidConfig(pidConfiguration pidConfig) {
  setPidTunings(pidConfig);
}

void applyPidAutoTuneConfig(pidAutoTuneConfiguration pidAutoTuneConfig) {
  setPidAutoTuneConfig(pidAutoTuneConfig);
}


void applyConfig(robotConfiguration config) {
  Serial.println("Apply Config");
  applyMotorConfig(config.motorConfig);
  applyMpuConfig(config.mpuConfig);
  applyPidConfig(config.pidConfig);
  applyPidAutoTuneConfig(config.pidAutoTuneConfig);
}
