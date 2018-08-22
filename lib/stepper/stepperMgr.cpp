#include "Arduino.h"
#include "stepperMgr.h"
#include "stepper.h"
#include "TeensyDelay.h"

StepperMgr mgr;

StepperMgr::StepperMgr() {
    TeensyDelay::begin();
    initMotorMgmt();
    setStepMode(MOTOR_THIRTYSECOND_STEP);
    enableMotors(false);
}

void StepperMgr::initMotorMgmt() {
  // setup direction for step mode pins.
  pinMode(MOTOR_STEP_MO_PIN, OUTPUT);
  pinMode(MOTOR_STEP_M1_PIN, OUTPUT);
  pinMode(MOTOR_STEP_M2_PIN, OUTPUT);

  // setup direction for motor enable pin.
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);

}
void StepperMgr::enableMotors(bool enabled) {
  digitalWrite(MOTOR_ENABLE_PIN, enabled?LOW:HIGH);
}

void StepperMgr::setStepMode(int stepMode) {
  _stepMode = stepMode;
  Serial.print("Stepping Mode:");
  Serial.println(stepMode);

  // write to stepping select pins.
  digitalWrite(MOTOR_STEP_MO_PIN, (stepMode & MOTOR_STEP_MO)?HIGH:LOW);
  digitalWrite(MOTOR_STEP_M1_PIN, (stepMode & MOTOR_STEP_M1)?HIGH:LOW);
  digitalWrite(MOTOR_STEP_M2_PIN, (stepMode & MOTOR_STEP_M2)?HIGH:LOW);
}

void StepperMgr::registerStepper(Stepper* stepper) {
    int channel = stepper->getChannel();
    _stepperLookup[channel] = stepper;
    fptr isr = StepperMgr::getStepDelayISR(channel);
    TeensyDelay::addDelayChannel(isr, channel);
}

fptr StepperMgr::getStepDelayISR(int channel) {
  //Serial.printf("getStepDelayISR:%d\n", channel);
  if (channel == 0) {
    //Serial.printf("doChannel0StepEnd\n");
    return StepperMgr::doChannel0StepEnd;
  }
  else {
    //Serial.printf("doChannel1StepEnd\n");
    return StepperMgr::doChannel1StepEnd;
  }
}

fptr StepperMgr::getStepIntervalISR(int channel) {
  //Serial.printf("getStepIntervalISR:%d\n", channel);
  if (channel == 0) {
    //Serial.printf("doChannel0Step\n");
    return StepperMgr::doChannel0Step;
  }
  else
  {
    //Serial.printf("doChannel1Step\n");
    return StepperMgr::doChannel1Step;
  }
}

double StepperMgr::calculateMicroseconds(double rps) {
  double microseconds = 0;
  double stepsPerSec = rps * (double)(MOTOR_STEPS_PER_REV * (2 << _stepMode));

  if (stepsPerSec < 0)
    stepsPerSec = 0-stepsPerSec;

  if (stepsPerSec > 0) {
    microseconds = 1000000 / stepsPerSec;
  }
  return microseconds;
}

Stepper* StepperMgr::getStepper(int channel) {
  return _stepperLookup[channel];
}

void StepperMgr::doChannelStepAction(int channel, bool start) {

  Stepper* stepper = mgr.getStepper(channel);
  if (start)
    stepper->handleStepStart();
  else
    stepper->handleStepEnd();
}
