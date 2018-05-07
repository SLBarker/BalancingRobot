#ifndef ROBOT_MPU
  #define ROBOT_MPU
  #define INTERRUPT_PIN 2


  #include "config.h"
  void setMpuOffsets(mpuCalibrationConfiguration mpuConfig);
  void initMpu();
  void processMpuData();
  void initAutoCalibrate();
  bool autoCalibrate(mpuCalibrationConfiguration * mpuConfig);
#endif
