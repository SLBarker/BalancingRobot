#ifndef ROBOT_MPU
  #define ROBOT_MPU
  #define MPU_SDA_PIN 8
  #define MPU_SCL_PIN 7
  #define INTERRUPT_PIN 12

  // index offsets for buffers
  #define AX  0
  #define AY  1
  #define AZ  2
  #define GX  3
  #define GY  4
  #define GZ  5
  #define COORDS  6

  #define IN_BALANCE_THRESHOLD 0.04
  #define BALANCE_LOST_THRESHOLD 0.3


extern int16_t raw[COORDS], mean[COORDS];
extern double pidSetpoint, pidInput, pidOutput;
extern bool motorTestMode;
extern float motorTestSpeed;
extern bool autotune;


  #include "config.h"
  void setMpuOffsets(mpuCalibrationConfiguration mpuConfig);
  void setPidTunings(pidConfiguration pidConfig);
  void setPidAutoTuneConfig(pidAutoTuneConfiguration pidAutoTuneConfig);
  void initMpu();
  void processMpuData();
  void initAutoCalibrate();
  bool autoCalibrate(mpuCalibrationConfiguration * mpuConfig);
#endif
