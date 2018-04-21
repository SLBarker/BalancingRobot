#include <Arduino.h>
#include <StepControl.h>
#include "motor.h"

Stepper motor_left(MOTOR_LEFT_STEP_PIN, MOTOR_LEFT_DIR_PIN);
Stepper motor_right(MOTOR_RIGHT_STEP_PIN, MOTOR_RIGHT_DIR_PIN);
StepControl<> controller_left;
StepControl<> controller_right;

void enableMotors(bool enable) {
  digitalWrite(MOTOR_ENABLE_PIN, enable?LOW:HIGH);
}

void initMotor() {
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  enableMotors(false);

  // configure motors
  motor_left.setPullInSpeed(MOTOR_PULL_IN_SPEED);
  motor_right.setPullInSpeed(MOTOR_PULL_IN_SPEED);
}

void setMotorSpeedLeft(float speed) {
  motor_left.setMaxSpeed(speed);
  controller_left.rotateAsync(motor_left);
}

void setMotorSpeedRight(float speed) {
  motor_right.setMaxSpeed(speed);
  controller_right.rotateAsync(motor_right);
}
