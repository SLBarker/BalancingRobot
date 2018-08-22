#include "Arduino.h"
#include "stepper.h"
#include "stepperMgr.h"

#ifndef ROBOT_MOTOR
  #define ROBOT_MOTOR

  extern StepperMgr mgr;
  extern Stepper motorLeft;
  extern Stepper motorRight;

#endif
