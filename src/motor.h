#ifndef ROBOT_MOTOR
  #define ROBOT_MOTOR

  #define MOTOR_ENABLE_PIN 21


  #define MOTOR_LEFT_DIR_PIN 34
  #define MOTOR_LEFT_STEP_PIN 35
  #define MOTOR_LEFT_SLEEP_PIN 36
  #define MOTOR_LEFT_RESET_PIN 37


  #define MOTOR_RIGHT_DIR_PIN 14
  #define MOTOR_RIGHT_STEP_PIN 15
  #define MOTOR_RIGHT_SLEEP_PIN 16
  #define MOTOR_RIGHT_RESET_PIN 17


  #define MOTOR_STEP_MO 1
  #define MOTOR_STEP_M1 2
  #define MOTOR_STEP_M2 4


  #define MOTOR_STEP_MO_PIN 20
  #define MOTOR_STEP_M1_PIN 19
  #define MOTOR_STEP_M2_PIN 18

  #define MOTOR_PULL_IN_SPEED 3000
  #define MOTOR_STEPS_PER_REV 200

  #define MOTOR_FULL_STEP 0
  #define MOTOR_HALF_STEP 1
  #define MOTOR_QUARTER_STEP 2
  #define MOTOR_EIGHTH_STEP 3
  #define MOTOR_SIXTEENTH_STEP 4
  #define MOTOR_THIRTYSECOND_STEP 5


  void initMotor();

  void enableMotors(bool enable);
  void setMotorStep(int stepMode);
  void setMotorSpeedLeft(float speed);
  void setMotorSpeedRight(float speed);
#endif
