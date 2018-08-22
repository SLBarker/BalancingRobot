#ifndef Stepper_h
#define Stepper_h

#include "Arduino.h"
#include "TeensyDelay.h"

typedef void (*fptr)();

#define DEFAULT_MAX_RPM 300.0
#define DEFAULT_MAX_ACCEL 3000
#define STEP_DURATION_MICROSECONDS 2
#define MOTOR_STEPS_PER_REV 200

#define MOTOR_LEFT_DIR_PIN 34
#define MOTOR_LEFT_STEP_PIN 35
#define MOTOR_LEFT_SLEEP_PIN 36
#define MOTOR_LEFT_RESET_PIN 37


#define MOTOR_RIGHT_DIR_PIN 14
#define MOTOR_RIGHT_STEP_PIN 15
#define MOTOR_RIGHT_SLEEP_PIN 16
#define MOTOR_RIGHT_RESET_PIN 17


class StepperMgr;

class Stepper
{
public:
  Stepper(int dirPin, int stepPin, int sleepPin, int resetPin, int channel, float maxRpm = DEFAULT_MAX_RPM, int maxAccel = DEFAULT_MAX_ACCEL);
  void handleStepStart();
  void handleStepEnd();
  int getChannel();
  void printDetails();
  void setSpeed(double rps);
private:
  int _dirPin;
  int _stepPin;
  int _sleepPin;
  int _resetPin;
  int _channel;
  float _maxRpm;
  int _maxAccel;
  float _currentRps;
  IntervalTimer _intervalTimer;
  fptr _intervalISR;
};

extern StepperMgr mgr;

#endif
