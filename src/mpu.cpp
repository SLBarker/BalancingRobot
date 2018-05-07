#include "I2Cdev.h"
#include <Arduino.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "mpu.h"
#include "motor.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
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
int16_t ax, ay, az,gx, gy, gz;
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz;

bool meansensors() {
  static int meanSensorIndex = 0;
  static long buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  // read raw accel/gyro measurements from device
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  if ((meanSensorIndex % 0) == 0) {
    // print raw values...
    Serial.println("");
    Serial.print("raw accl: (");
    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.print(az);
    Serial.println(")");

    Serial.print("raw gyro: (");
    Serial.print(gx);
    Serial.print(",");
    Serial.print(gy);
    Serial.print(",");
    Serial.print(gz);
    Serial.println(")");
  }

  //Serial.print("m");
  if (meanSensorIndex >=100 && meanSensorIndex < (100 + buffersize) ) {

    buff_ax = buff_ax + ax;
    buff_ay = buff_ay + ay;
    buff_az = buff_az + az;
    buff_gx = buff_gx + gx;
    buff_gy = buff_gy + gy;
    buff_gz = buff_gz + gz;
  }
  if (meanSensorIndex == (100 + buffersize) ) {
    mean_ax = buff_ax / buffersize;
    mean_ay = buff_ay / buffersize;
    mean_az = buff_az / buffersize;
    mean_gx = buff_gx / buffersize;
    mean_gy = buff_gy / buffersize;
    mean_gz = buff_gz / buffersize;

    // print means...
    Serial.println("");
    Serial.print("mean accl: (");
    Serial.print(mean_ax);
    Serial.print(",");
    Serial.print(mean_ay);
    Serial.print(",");
    Serial.print(mean_az);
    Serial.println(")");

    Serial.print("mean gyro: (");
    Serial.print(mean_gx);
    Serial.print(",");
    Serial.print(mean_gy);
    Serial.print(",");
    Serial.print(mean_gz);
    Serial.println(")");


    // reset for next meansensor run
    meanSensorIndex = 0;
    buff_ax = 0;
    buff_ay = 0;
    buff_az = 0;
    buff_gx = 0;
    buff_gy = 0;
    buff_gz = 0;
    return true;
  }
  else
      meanSensorIndex++;
  return false;
}

bool calibration(mpuCalibrationConfiguration * mpuConfig) {

    int ready=0;
    //Serial.print("c");
    if (meansensors()) {

      if (abs(mean_ax) <= acel_deadzone) {
        ready++;
      }
      else
        mpuConfig->xAccelOffset=mpuConfig->xAccelOffset-mean_ax/acel_deadzone;

      if (abs(mean_ay) <= acel_deadzone) {
        ready++;
      }
      else
        mpuConfig->yAccelOffset=mpuConfig->yAccelOffset-mean_ay/acel_deadzone;

      if (abs(16384-mean_az) <= acel_deadzone) {
        ready++;
      }
      else
        mpuConfig->zAccelOffset=mpuConfig->zAccelOffset+(16384-mean_az)/acel_deadzone;

      if (abs(mean_gx) <= giro_deadzone) {
        ready++;
      }
      else
        mpuConfig->xGyroOffset=mpuConfig->xGyroOffset-mean_gx/(giro_deadzone+1);

      if (abs(mean_gy) <= giro_deadzone) {
        ready++;
      }
      else
        mpuConfig->yGyroOffset=mpuConfig->yGyroOffset-mean_gy/(giro_deadzone+1);

      if (abs(mean_gz) <= giro_deadzone) {
        ready++;
      }
      else
        mpuConfig->zGyroOffset=mpuConfig->zGyroOffset-mean_gz/(giro_deadzone+1);


      setMpuOffsets(*mpuConfig);
      Serial.println("");
      Serial.print("offset accl: (");
      Serial.print(mpuConfig->xAccelOffset);
      Serial.print(",");
      Serial.print(mpuConfig->yAccelOffset);
      Serial.print(",");
      Serial.print(mpuConfig->zAccelOffset);
      Serial.println(")");

      Serial.print("offset gyro: (");
      Serial.print(mpuConfig->xGyroOffset);
      Serial.print(",");
      Serial.print(mpuConfig->yGyroOffset);
      Serial.print(",");
      Serial.print(mpuConfig->zGyroOffset);
      Serial.println(")");
    }
    return (ready == 6);
}

bool autoCalibrate(mpuCalibrationConfiguration * mpuConfig) {

  static int calibrateState=0;

  switch (calibrateState) {
    case 0:
      Serial.println("State 0: Calibrate init...");
      detachInterrupt(INTERRUPT_PIN);
      //mpu.reset();
      //mpu.initialize();
      //mpu.setDMPEnabled(false);
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
        mpuConfig->xAccelOffset=-mean_ax/8;
        mpuConfig->yAccelOffset=-mean_ay/8;
        mpuConfig->zAccelOffset=(16384-mean_az)/8;

        mpuConfig->xGyroOffset=-mean_gx/4;
        mpuConfig->yGyroOffset=-mean_gy/4;
        mpuConfig->zGyroOffset=-mean_gz/4;
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
        // re-enable interrupts
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), processMpuData, RISING);
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
  //Serial.print(".");
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

      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      setMotorSpeedLeft(ypr[2] * 15000);
      setMotorSpeedRight(ypr[1] * 15000);
  }
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
