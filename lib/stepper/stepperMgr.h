#ifndef StepperMgr_h
#define StepperMgr_h

#include "Arduino.h"
#include "stepper.h"

#define MAX_CHANNELS  2

#define MOTOR_FULL_STEP 0
#define MOTOR_HALF_STEP 1
#define MOTOR_QUARTER_STEP 2
#define MOTOR_EIGHTH_STEP 3
#define MOTOR_SIXTEENTH_STEP 4
#define MOTOR_THIRTYSECOND_STEP 5

#define MOTOR_STEP_MO 1
#define MOTOR_STEP_M1 2
#define MOTOR_STEP_M2 4

#define MOTOR_ENABLE_PIN 21
#define MOTOR_STEP_MO_PIN 20
#define MOTOR_STEP_M1_PIN 19
#define MOTOR_STEP_M2_PIN 18

class StepperMgr
{
public:
  //static StepperMgr* getInstance();
  // ISR's
  StepperMgr();
  static void doChannelStepAction(int channel, bool start);
  static inline void doChannel0Step() { doChannelStepAction(0, true); }
  static inline void doChannel0StepEnd() { doChannelStepAction(0, false); }
  static inline void doChannel1Step() { doChannelStepAction(1, true); }
  static inline void doChannel1StepEnd() { doChannelStepAction(1, false); }
  static fptr getStepIntervalISR(int channel);
  static fptr getStepDelayISR(int channel);

  static inline void dummyISR() {}

  void enableMotors(bool enable);
  void setStepMode(int stepMode);
  void registerStepper(Stepper* stepper);

  Stepper* getStepper(int channel);
  double calculateMicroseconds(double rps);


private:
  //StepperMgr();
  float rpmToStepsPerSec(float rpm);
  void initMotorMgmt();

  //static StepperMgr* _instance;
  Stepper* _stepperLookup[MAX_CHANNELS];
  int _stepMode;
};
#endif
