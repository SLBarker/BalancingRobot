#ifndef ROBOT_CONFIG
  #define ROBOT_CONFIG

  #define CONFIG_VERSION 8
  #include <Arduino.h>

  typedef struct
  {
    int xGyroOffset;
    int yGyroOffset;
    int zGyroOffset;
    int xAccelOffset;
    int yAccelOffset;
    int zAccelOffset;
  } mpuCalibrationConfiguration;

  typedef struct
  {
    bool enabled;
    int stepMode;
  } motorConfiguration;

  typedef struct
  {
    double kp;
    double ki;
    double kd;

  } pidConfiguration;

  typedef struct
  {
    double outputStep;
    int controlType;
    int lookbackSec;
    double noiseBand;
  } pidAutoTuneConfiguration;

  typedef struct
  {
    int version;
    motorConfiguration motorConfig;
    pidConfiguration pidConfig;
    mpuCalibrationConfiguration mpuConfig;
    pidAutoTuneConfiguration pidAutoTuneConfig;
  } robotConfiguration;


  robotConfiguration createDefaultConfig();
  robotConfiguration readConfig();
  bool writeConfig(robotConfiguration);
  void applyConfig(robotConfiguration config);
  void applyPidConfig(pidConfiguration pidConfig);

  extern robotConfiguration robotConfig;
#endif
