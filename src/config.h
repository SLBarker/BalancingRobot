#ifndef ROBOT_CONFIG
  #define ROBOT_CONFIG

  #define CONFIG_VERSION 5
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
    float potential;
    float integral;
    float derivative;

  } pidConfiguration;

  typedef struct
  {
    int version;
    motorConfiguration motorConfig;
    pidConfiguration pidConfig;
    mpuCalibrationConfiguration mpuConfig;
  } robotConfiguration;


  robotConfiguration createDefaultConfig();
  robotConfiguration readConfig();
  bool writeConfig(robotConfiguration);
  void applyConfig(robotConfiguration config);
#endif
