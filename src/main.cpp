#include "I2Cdev.h"
#include <Arduino.h>
#include <SPI.h>

#include <SSD_13XX.h>
#include <GoBLE.h>
#include <menu.h>
#include <menuIO/ssd1331Out.h>
#include <menuIO/chainStream.h>
#include <menuIO/serialOut.h>
#include <menuIO/serialIn.h>
#include <menuIO/stringIn.h>
#include <StepControl.h>
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

using namespace Menu;

#define INTERRUPT_PIN 2
#define OLED_CS_PIN 15
#define OLED_DC_PIN 26
#define OLED_RST_PIN 255
#define OLED_MOSI_PIN 11
#define OLED_SCLK_PIN 14
#define LED_PIN 13
#define MOTOR_ENABLE_PIN 16

#define BLUETOOTH_SERIAL Serial5

#define OUTPUT_READABLE_YAWPITCHROLL

MPU6050 mpu;
SSD_13XX gfx(OLED_CS_PIN, OLED_DC_PIN, OLED_RST_PIN, OLED_MOSI_PIN, OLED_SCLK_PIN);
_GoBLE goble(&BLUETOOTH_SERIAL);


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
// VectorInt16 aa;         // [x, y, z]            accel sensor measurements
// VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
// VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

Stepper motor_left(24, 25);         // STEP pin: 24, DIR pin: 25
Stepper motor_right(27, 28);         // STEP pin: 11, DIR pin: 12
StepControl<> controller_left;    // Use default settings
StepControl<> controller_right;

int test=55;
int ledCount=0;
int motorCtrl=HIGH;

result doAlert(eventMask e, prompt &item);



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

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
          // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      //    displayYPRBar(48, 20, 47, 15, RED, ypr[0] * 180/M_PI);
      //    displayYPRBar(48, 35, 47, 15, GREEN, ypr[1] *180/M_PI);
      //      displayYPRBar(48, 50, 47, 15, BLUE, ypr[2] *180/M_PI);

         motor_right.setMaxSpeed(ypr[1] * 15000);  // Set target position to 1000 steps from current position
         motor_left.setMaxSpeed(ypr[2] * 15000);
         controller_left.rotateAsync(motor_left);    // Do the move
         controller_right.rotateAsync(motor_right);
      #endif

  }
}

//void dmpDataReady() {
//    processMpuData();
//}

result doToggleMotorCtrl() {
  Serial.println("Motor Toggle");
  digitalWrite(MOTOR_ENABLE_PIN, motorCtrl);
  return proceed;
}

navCodesDef myNavCodes={
  {noCmd,(char)0xff},
  {escCmd,'/'},
  {enterCmd,'*'},
  {upCmd,'-'},
  {downCmd,'+'},
  {leftCmd,'<'},
  {rightCmd,'>'},
  {idxCmd,'H'},
  {scrlUpCmd,0x35},
  {scrlUpCmd,0x36}
};

config myOptions('*', '-', myNavCodes, false);

TOGGLE(motorCtrl,setMotor,"Motor Control: ",doNothing,noEvent,noStyle//,doExit,enterEvent,noStyle
  ,VALUE("Off", HIGH, doToggleMotorCtrl, enterEvent)
  ,VALUE("On", LOW, doToggleMotorCtrl, enterEvent)
);

int selTest=0;
SELECT(selTest,selMenu,"Select Test",doNothing,noEvent,noStyle
  ,VALUE("Zero",0,doNothing,noEvent)
  ,VALUE("One",1,doNothing,noEvent)
  ,VALUE("Two",2,doNothing,noEvent)
);

int chooseTest=-1;
CHOOSE(chooseTest,chooseMenu,"Choose Test",doNothing,noEvent,noStyle
  ,VALUE("First",1,doNothing,noEvent)
  ,VALUE("Second",2,doNothing,noEvent)
  ,VALUE("Third",3,doNothing,noEvent)
  ,VALUE("Last",-1,doNothing,noEvent)
);

//customizing a prompt look!
//by extending the prompt class
class altPrompt:public prompt {
public:
  altPrompt(constMEM promptShadow& p):prompt(p) {}
  Used printTo(navRoot &root,bool sel,menuOut& out, idx_t idx,idx_t len,idx_t) override {
    return out.printRaw("special prompt!",len);;
  }
};

MENU(subMenu,"Sub-Menu",doNothing,noEvent,noStyle
  ,altOP(altPrompt,"",doNothing,noEvent)
  ,OP("Op",doNothing,noEvent)
  ,EXIT("<Back")
);

char* constMEM hexDigit MEMMODE="0123456789ABCDEF";
char* constMEM hexNr[] MEMMODE={"0","x",hexDigit,hexDigit};
char buf1[]="0x11";

MENU(mainMenu,"Robot Control Menu",doNothing,noEvent,wrapStyle
  ,SUBMENU(subMenu)
  ,SUBMENU(setMotor)
  //,OP("Motor En On",motorControlOn,enterEvent)
  //,OP("Motor En Off",motorControlOff,enterEvent)
  ,SUBMENU(selMenu)
  ,SUBMENU(chooseMenu)
  //,OP("Alert test",doAlert,enterEvent)
  ,FIELD(test,"Test","%",0,100,10,1,doNothing,noEvent,wrapStyle)
  //,EDIT("Hex",buf1,hexNr,doNothing,noEvent,noStyle)
  ,EXIT("<Exit")
);


// define menu colors --------------------------------------------------------
//  {{disabled normal,disabled selected},{enabled normal,enabled selected, enabled editing}}
//monochromatic color table

#define GRAY RGB565(128,128,128)

const colorDef<uint16_t> colors[] MEMMODE={
  {{BLACK,BLACK},{BLACK,BLUE,BLUE}},//bgColor
  {{GRAY,GRAY},{WHITE,WHITE,WHITE}},//fgColor
  {{WHITE,BLACK},{YELLOW,YELLOW,RED}},//valColor
  {{WHITE,BLACK},{WHITE,YELLOW,YELLOW}},//unitColor
  {{WHITE,GRAY},{BLACK,BLUE,WHITE}},//cursorColor
  {{WHITE,YELLOW},{BLACK,RED,RED}},//titleColor
};

stringIn<4> strIn; //buffer size: 2^5 = 32 bytes, eventually use 0 for a single byte
serialIn serial(Serial);
MENU_INPUTS(in,&strIn);

#define MAX_DEPTH 4
#define textScale 1
MENU_OUTPUTS(out,MAX_DEPTH
  ,SSD1331_OUT(gfx,colors,6*textScale,9*textScale,{0,0,25,6})
  //  ,SSD1331_OUT(gfx,colors,6*textScale,9*textScale,{0,0,14,8},{14,0,14,8})
  ,SERIAL_OUT(Serial)
);

NAVROOT(nav,mainMenu,MAX_DEPTH,in,out);

//when menu is suspended
result idle(menuOut& o,idleEvent e) {
  if (e==idling) {
    o.println(F("suspended..."));
    o.println(F("press [select]"));
    o.println(F("to continue"));
  }
  return proceed;
}

void initMenu() {
  options=&myOptions;
}

void setup() {
  Serial.begin(115200);
  BLUETOOTH_SERIAL.begin(9600);

  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN, motorCtrl);


  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // configure motors
  motor_left.setPullInSpeed(3000);
  motor_right.setPullInSpeed(3000);


  pinMode(LED_PIN, OUTPUT);


  initMenu();

  gfx.begin(false);
  gfx.setRotation(2);
  gfx.setTextScale(textScale);
  gfx.setTextWrap(false);
  gfx.fillScreen(BLACK);
  gfx.setTextColor(RED,BLACK);
  gfx.println("Setup...");
  Serial.flush();
  nav.idleTask=idle;//point a function to be used when menu is suspended


  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-19);
  mpu.setYGyroOffset(19);
  mpu.setZGyroOffset(2160);
  mpu.setZAccelOffset(2160); // 1688 factory default for my test chip

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
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
  //delay(1000);
  Serial.println("Setup Complete");
}

void fadeLed() {
  ledCount++;
  int phase = (ledCount >> 8) & 0xff;
  if (phase >= 0x80) {
    phase = 0xff - phase;
  }

  if ((ledCount % phase) == 0)
    digitalWrite(LED_PIN, HIGH);
  else
    digitalWrite(LED_PIN, LOW);
}


void checkForNavInput(stringIn<4u>& in) {

  int ch = -1;
  if (goble.available()) {
    Serial.println("input detected");
    if (goble.readSwitchUp() == PRESSED)
      ch = '+';

    if (goble.readSwitchDown() == PRESSED)
      ch = '-';

    if (goble.readSwitchLeft() == PRESSED)
      ch = '<';

    if (goble.readSwitchRight() == PRESSED)
      ch = '>';

    if (goble.readSwitchSelect() == PRESSED)
      ch = '/';

    if (goble.readSwitchStart() == PRESSED)
      ch = '*';
  }
  if (ch != -1) {
    in.write(ch);
  }
}


void loop() {
  // if programming failed, don't try to do anything
  // if (!dmpReady) return;
  //Serial.println("looping");
  // wait for MPU interrupt or extra packet(s) available
  //while (!mpuInterrupt && fifoCount < packetSize) {
    //Serial.println("start poll");
    checkForNavInput(strIn);
    nav.poll();//this device only draws when needed
    fadeLed();
    delay(1);
  //}
}
