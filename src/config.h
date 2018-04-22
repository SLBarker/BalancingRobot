#ifndef ROBOT_CONFIG
  #define ROBOT_CONFIG

  #define CONFIG_VERSION 1

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
  } robotConfiguration;


  robotConfiguration createDefaultConfig();
  robotConfiguration readConfig();
  bool writeConfig(robotConfiguration);
  void applyConfig(robotConfiguration config);
#endif
