#include "I2Cdev.h"
#include <Arduino.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <stepperMgr.h>
#include <stepper.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "mpu.h"
#include "motor.h"
#include "display.h"
#include "input.h"

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
    P_ON_M,
    DIRECT);

PID_ATune aTune(&pidInput, &pidOutput);

MPU6050 mpu;

// MPU control/status vars

bool autotune = false;
bool tuning = false;
bool rawMpuAvailable = false;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
// uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t packetSize;    // expected DMP packet size (default is 42 bytes)

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

//long nextMpuOut =0;
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
  Serial.printf("xGyroOffset:%f\n", mpuConfig.xGyroOffset);
  mpu.setXGyroOffset(mpuConfig.xGyroOffset);
  Serial.printf("yGyroOffset:%f\n", mpuConfig.yGyroOffset);
  mpu.setYGyroOffset(mpuConfig.yGyroOffset);
  Serial.printf("zGyroOffset:%f\n", mpuConfig.zGyroOffset);
  mpu.setZGyroOffset(mpuConfig.zGyroOffset);

  Serial.printf("xAccelOffset:%f\n", mpuConfig.xAccelOffset);
  mpu.setXAccelOffset(mpuConfig.xAccelOffset);
  Serial.printf("yAccelOffset:%f\n", mpuConfig.yAccelOffset);
  mpu.setYAccelOffset(mpuConfig.yAccelOffset);
  Serial.printf("zAccelOffset:%f\n", mpuConfig.zAccelOffset);
  mpu.setZAccelOffset(mpuConfig.zAccelOffset);
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
void processMpuData() {

  static byte iteration=0;

  // get INT_STATUS byte
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  uint16_t fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)

    while (fifoCount >= packetSize) {
    //  Serial.printf("fifo:%d\n", fifoCount);
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }

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

    double joyX = ((double)readJoystickX())/1300;
    double joyY = ((double)readJoystickY())/300;
    pidInput = ypr[1];
    pidSetpoint = joyX;
    double balanceError = abs(pidInput - pidSetpoint);

    if (!autotune) {
      switch (pid.GetMode()) {
        case MANUAL:
          if (balanceError < IN_BALANCE_THRESHOLD)
            pid.SetMode(AUTOMATIC);
          break;

        case AUTOMATIC:
          if (balanceError > BALANCE_LOST_THRESHOLD)
            pid.SetMode(MANUAL);
          break;
      }

      if (pid.GetMode() == AUTOMATIC) {
        bool res = pid.Compute();

        ++iteration;
        if (iteration == 40) {
          iteration = 0;
          Serial.printf("joyx:%f joyy:%f\n", joyX, joyY);

          Serial.printf("time: %d\n", millis());
          Serial.printf("Kp:%.4f, Ki:%.4f, Kd:%.4f\n", pid.GetKp(), pid.GetKi(), pid.GetKd());
          Serial.printf("pidInput:%f pidSetpoint:%f\n", pidInput, pidSetpoint);
          Serial.printf("balanceError:%f pidOuput:%f\n", balanceError, pidOutput);
        }

        if (motorTestMode) {
          motorTestSpeed += joyY ;
          motorLeft.setSpeed(-motorTestSpeed);
          motorRight.setSpeed(motorTestSpeed);
        } else {
          motorLeft.setSpeed(-pidOutput+joyY);
          motorRight.setSpeed(pidOutput+joyY);
        }
      } else {
        motorLeft.setSpeed(0);
        motorRight.setSpeed(0);
      }
    } else {

      pid.SetMode(MANUAL);
      // auto-tuning calls.
      if (!tuning) {
        Serial.printf("Waiting for Autotune...:\n");
        // need to wait until we are approximately balanced before starting...
        if (balanceError < IN_BALANCE_THRESHOLD) {
          Serial.printf("Starting Autotune...:\n");
          tuning = true;
        }
      }
      if (tuning) {
        if (aTune.Runtime() != 0) {
          Serial.printf("Autotune complete\n");
          // tuning complete...
          autotune = false;
          tuning = false;

          Serial.printf("Autotune results - Kp:%f Ki:%f Kd:%f\n", aTune.GetKp(), aTune.GetKi(), aTune.GetKd());
          // apply the updated Pid configuraion...
          robotConfig.pidConfig.kp = aTune.GetKp();
          robotConfig.pidConfig.ki = aTune.GetKi();
          robotConfig.pidConfig.kd = aTune.GetKd();
          applyPidConfig(robotConfig.pidConfig);
        }
      }
    }
  }
}

void setPidTunings(pidConfiguration pidConfig) {
  Serial.printf("Set Pid k=%f, i=%.4f, d=%f\n", abs(pidConfig.kp)/10, abs(pidConfig.ki)/10, abs(pidConfig.kd)/10);
  pid.SetTunings(abs(pidConfig.kp)/10, abs(pidConfig.ki)/10, abs(pidConfig.kd)/10, P_ON_M);
  Serial.printf("Kp:%f, Ki:%.4f, Kd:%f\n", pid.GetKp(), pid.GetKi(), pid.GetKd());

}

void setPidAutoTuneConfig(pidAutoTuneConfiguration pidAutoTuneConfig) {
  Serial.printf("Set Auto Tune settings\n");
  aTune.SetOutputStep(pidAutoTuneConfig.outputStep);
  aTune.SetControlType(pidAutoTuneConfig.controlType);
  aTune.SetLookbackSec(pidAutoTuneConfig.lookbackSec);
  aTune.SetNoiseBand(pidAutoTuneConfig.noiseBand);
}

void initMpu() {
  // initialize device
  Serial.println(F("Initializing I2C devices..."));

  Wire.setSDA(MPU_SDA_PIN);
  Wire.setSCL(MPU_SCL_PIN);


  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  pid.SetSampleTime(10);
  pid.SetMode(MANUAL);
  pid.SetOutputLimits(-5,5);
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  uint8_t devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.println(F("Enabling interrupt detection for MPU dats available..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), processMpuData, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
}
