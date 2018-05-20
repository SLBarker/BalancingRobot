#include "I2Cdev.h"
#include <Arduino.h>
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "mpu.h"
#include "motor.h"
#include "display.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// PID controller
double pidSetpoint, pidInput, pidOutput, lastOutput;

PID pid(
    &pidInput,
    &pidOutput,
    &pidSetpoint,
    0,
    0,
    0,
    DIRECT);

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
bool rawMpuAvailable = false;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// auto calibrate variables....

// Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// calibration working variables...
int16_t raw[COORDS], mean[COORDS];

bool meansensors() {
  static int meanSensorIndex = 0;
  static long buff[COORDS];

  if (rawMpuAvailable) {
    if ((meanSensorIndex % 0) == 0) {
      // print raw values...
      //printMpu(raw);
      Serial.println("");
      Serial.printf("raw accl: (%d, %d, %d)\n", raw[AX], raw[AY], raw[AZ]);
      Serial.printf("raw gyro: (%d, %d, %d)\n", raw[GX], raw[GY], raw[GZ]);
    }

    if (meanSensorIndex >=100 && meanSensorIndex < (100 + buffersize) ) {
      for (int i=0; i<COORDS; i++) {
        buff[i] = buff[i] + raw[i];
      }
    }

    if (meanSensorIndex == (100 + buffersize) ) {
      for (int i=0; i<COORDS; i++) {
        mean[i] = buff[i] / buffersize;
      }

      // print means...
      //printMpu(mean);
      Serial.println("");
      Serial.printf("mean accl: (%d, %d, %d)\n", mean[AX], mean[AY], mean[AZ]);
      Serial.printf("mean gyro: (%d, %d, %d)\n", mean[GX], mean[GY], mean[GZ]);

      // reset for next meansensor run
      meanSensorIndex = 0;
      for (int i=0; i<COORDS; i++) {
        buff[i] = 0;
      }
      return true;
    }
    else
        meanSensorIndex++;

    // clear latch - to allow next raw value to be read.
    rawMpuAvailable = false;
  }
  return false;
}

bool calibration(mpuCalibrationConfiguration * mpuConfig) {

    int ready=0;
    if (meansensors()) {

      if (abs(mean[AX]) <= acel_deadzone)
        ready++;
      else
        mpuConfig->xAccelOffset=mpuConfig->xAccelOffset-mean[AX]/acel_deadzone;

      if (abs(mean[AY]) <= acel_deadzone)
        ready++;
      else
        mpuConfig->yAccelOffset=mpuConfig->yAccelOffset-mean[AY]/acel_deadzone;

      if (abs(16384-mean[AZ]) <= acel_deadzone)
        ready++;
      else
        mpuConfig->zAccelOffset=mpuConfig->zAccelOffset+(16384-mean[AZ])/acel_deadzone;

      if (abs(mean[GX]) <= giro_deadzone)
        ready++;
      else
        mpuConfig->xGyroOffset=mpuConfig->xGyroOffset-mean[GX]/(giro_deadzone+1);

      if (abs(mean[GY]) <= giro_deadzone)
        ready++;
      else
        mpuConfig->yGyroOffset=mpuConfig->yGyroOffset-mean[GY]/(giro_deadzone+1);

      if (abs(mean[GZ]) <= giro_deadzone)
        ready++;
      else
        mpuConfig->zGyroOffset=mpuConfig->zGyroOffset-mean[GZ]/(giro_deadzone+1);


      setMpuOffsets(*mpuConfig);
      Serial.println("");
      Serial.printf("offset accl: (%d, %d, %d)\n",
        mpuConfig->xAccelOffset,
        mpuConfig->yAccelOffset,
        mpuConfig->zAccelOffset);

      Serial.printf("offset gyro: (%d, %d, %d)\n",
        mpuConfig->xGyroOffset,
        mpuConfig->yGyroOffset,
        mpuConfig->zGyroOffset);
    }
    return (ready == 6);
}

bool autoCalibrate(mpuCalibrationConfiguration * mpuConfig) {

  static int calibrateState=0;

  switch (calibrateState) {
    case 0:
      Serial.println("State 0: Calibrate init...");
      // start of calibrate
      // reset offsets and apply the values
      mpuConfig->xAccelOffset=0;
      mpuConfig->yAccelOffset=0;
      mpuConfig->zAccelOffset=0;

      mpuConfig->xGyroOffset=0;
      mpuConfig->yGyroOffset=0;
      mpuConfig->zGyroOffset=0;
      setMpuOffsets(*mpuConfig);
      calibrateState++;
      break;

    case 1:

      if (meansensors()) {
        Serial.println("State 1: meansensors complete...");
        calibrateState++;
        mpuConfig->xAccelOffset=-mean[AX]/8;
        mpuConfig->yAccelOffset=-mean[AY]/8;
        mpuConfig->zAccelOffset=(16384-mean[AZ])/8;

        mpuConfig->xGyroOffset=-mean[GX]/4;
        mpuConfig->yGyroOffset=-mean[GY]/4;
        mpuConfig->zGyroOffset=-mean[GZ]/4;
        setMpuOffsets(*mpuConfig);
      }
      break;

    case 2:
      // perform calibration...
      if (calibration(mpuConfig)) {
        Serial.println("State 2: Calibration complete...");
        calibrateState++;
      }
      break;

    case 3:
      // finalise...
      if (meansensors()) {
        Serial.println("State 3: Compute final offsets...");
        calibrateState = 0;
      }
      break;
  }

  return calibrateState == 0;
}

void setMpuOffsets(mpuCalibrationConfiguration mpuConfig) {
  mpu.setXGyroOffset(mpuConfig.xGyroOffset);
  mpu.setYGyroOffset(mpuConfig.yGyroOffset);
  mpu.setZGyroOffset(mpuConfig.zGyroOffset);

  mpu.setXAccelOffset(mpuConfig.xAccelOffset);
  mpu.setYAccelOffset(mpuConfig.yAccelOffset);
  mpu.setZAccelOffset(mpuConfig.zAccelOffset);
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
void processMpuData() {

  // get INT_STATUS byte
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // read and "latch" new raw values if needed (used for calibration)
      if (!rawMpuAvailable)
      {
        mpu.getMotion6(&raw[AX], &raw[AY], &raw[AZ], &raw[GX], &raw[GY], &raw[GZ]);
        rawMpuAvailable = true;
      }
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      pidInput = ypr[1]*100;
      pid.Compute();

      setMotorSpeedLeft(pidOutput);
      setMotorSpeedRight(pidOutput);
  }
}

void setPidTunings(pidConfiguration pidConfig) {
  pid.SetTunings(pidConfig.kp, pidConfig.ki, pidConfig.kd);
  Serial.printf("Set Pid k=%f, i=%f, d=%f\n", pidConfig.kp, pidConfig.ki, pidConfig.kd);
}

void initMpu() {
  // initialize device
  Serial.println(F("Initializing I2C devices..."));

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  pid.SetSampleTime(10);
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-50000,50000);
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), processMpuData, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
}
