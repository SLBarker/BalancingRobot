#include "Arduino.h"
#include "stepper.h"
#include "stepperMgr.h"

Stepper::Stepper(int dirPin, int stepPin, int sleepPin, int resetPin, int channel, float maxRpm, int maxAccel)
{
  // configure core motor pins
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);

  if (sleepPin != -1) {
    pinMode(sleepPin, OUTPUT);
    digitalWrite(sleepPin, HIGH);
  }

  if (resetPin != -1) {
    pinMode(resetPin, OUTPUT);
    digitalWrite(resetPin, HIGH);
  }

  _dirPin = dirPin;
  _stepPin = stepPin;
  _sleepPin = sleepPin;
  _resetPin = resetPin;
  _channel = channel;
  _maxRpm = maxRpm;
  _maxAccel = maxAccel;
  _currentRps = -1;

  //StepperMgr* mgr = StepperMgr::getInstance();
  //_intervalISR = mgr->getStepIntervalISR(channel);

  _intervalISR = mgr.getStepIntervalISR(channel);
}

// called on start of step (rising step edge).
// trigger small timing delay before step end.
void Stepper::handleStepStart() {
  // set step pin
  digitalWriteFast(_stepPin, HIGH);
  // setup delay timer to complete step.
  TeensyDelay::trigger(STEP_DURATION_MICROSECONDS, _channel);
}

// called on end of step (falling step edge).
void Stepper::handleStepEnd() {
  // clear step pin
  digitalWriteFast(_stepPin, LOW);
}

void Stepper::setSpeed(double rps) {
  Serial.printf("setSpeed:%f\n", rps);
  //StepperMgr* mgr = StepperMgr::getInstance();
  double microseconds = mgr.calculateMicroseconds(rps);
  Serial.printf("micro:%f\n", microseconds);

  // set direction....
  digitalWriteFast(_dirPin, (rps >0)?HIGH:LOW);
  if (microseconds != 0) {
    // update intervaltimer
    _intervalTimer.begin(_intervalISR, microseconds);
  } else {
    _intervalTimer.end();
  }

  _currentRps = rps;
}

void Stepper::printDetails() {
  Serial.printf("dir:%d\n", _dirPin);
  Serial.printf("step:%d\n", _stepPin);
  Serial.printf("sleep:%d\n", _sleepPin);
  Serial.printf("reset:%d\n", _resetPin);
  Serial.printf("channel:%d\n", _channel);
}

int Stepper::getChannel() {
  return _channel;
}
