#include <Arduino.h>
#include <StepControl.h>
#include "motor.h"

Stepper motor_left(MOTOR_LEFT_STEP_PIN, MOTOR_LEFT_DIR_PIN);
Stepper motor_right(MOTOR_RIGHT_STEP_PIN, MOTOR_RIGHT_DIR_PIN);
StepControl<> controller_left;
StepControl<> controller_right;

void enableMotors(bool enabled) {
  digitalWrite(MOTOR_ENABLE_PIN, enabled?LOW:HIGH);
}

void setMotorStep(int stepMode) {
    Serial.print("Stepping Mode:");
    Serial.println(stepMode);

    digitalWrite(MOTOR_STEP_MO_PIN, (stepMode & MOTOR_STEP_MO)?HIGH:LOW);
    digitalWrite(MOTOR_STEP_M1_PIN, (stepMode & MOTOR_STEP_M1)?HIGH:LOW);
    digitalWrite(MOTOR_STEP_M2_PIN, (stepMode & MOTOR_STEP_M2)?HIGH:LOW);
}

void initMotor() {
  // configure the pin used for master motor enable.
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);

  // configure left motor pins
  pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_STEP_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_SLEEP_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_RESET_PIN, OUTPUT);

  digitalWrite(MOTOR_LEFT_SLEEP_PIN, HIGH);
  digitalWrite(MOTOR_LEFT_RESET_PIN, HIGH);

  // configure right motor pins
  pinMode(MOTOR_RIGHT_DIR_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_STEP_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_SLEEP_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_RESET_PIN, OUTPUT);

  digitalWrite(MOTOR_RIGHT_SLEEP_PIN, HIGH);
  digitalWrite(MOTOR_RIGHT_RESET_PIN, HIGH);

  // configure the pins used to control the stepping mode.
  pinMode(MOTOR_STEP_MO_PIN, OUTPUT);
  pinMode(MOTOR_STEP_M1_PIN, OUTPUT);
  pinMode(MOTOR_STEP_M2_PIN, OUTPUT);

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
