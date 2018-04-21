#ifndef ROBOT_CONFIG
  #define ROBOT_CONFIG

  #define CONFIG_VERSION 1

  enum stepping {
    fullStep = 0,
    halfStep = 4,
    quarterStep = 2,
    eighthStep = 6,
    sixteenthStep = 1,
    thirtySecondStep = 5
  };

  typedef struct
  {
    bool enable;
    enum stepping stepMode;
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


#endif
