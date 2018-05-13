#ifndef ROBOT_MPU
  #define ROBOT_MPU
  #define INTERRUPT_PIN 2

  // index offsets for buffers
  #define AX  0
  #define AY  1
  #define AZ  2
  #define GX  3
  #define GY  4
  #define GZ  5
  #define COORDS  6


extern int16_t raw[COORDS], mean[COORDS];
extern double pidSetpoint, pidInput, pidOutput;


  #include "config.h"
  void setMpuOffsets(mpuCalibrationConfiguration mpuConfig);
  void setPidTunings(pidConfiguration pidConfig);
  void initMpu();
  void processMpuData();
  void initAutoCalibrate();
  bool autoCalibrate(mpuCalibrationConfiguration * mpuConfig);
#endif
