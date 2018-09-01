#include "I2Cdev.h"
#include <Arduino.h>
#include <SPI.h>

#include "SSD_13XX.h"
#include <GoBLE.h>
#include <menu.h>
#include <menuIO/ssd1331Out.h>
#include <menuIO/chainStream.h>
#include <menuIO/serialOut.h>
#include <menuIO/serialIn.h>
#include <menuIO/stringIn.h>
#include <stepperMgr.h>
#include <stepper.h>
#include "motor.h"
#include "config.h"
#include "mpu.h"
#include "pid.h"
#include "battery.h"
#include "input.h"

using namespace Menu;

#define OLED_CS_PIN 9
#define OLED_DC_PIN 26
#define OLED_RST_PIN 29
#define OLED_MOSI_PIN 28
#define OLED_SCLK_PIN 27

#define LED_PIN 13

#define MENU_TEXT_SCALE 1
#define MAX_DEPTH 4
#define GRAY RGB565(32,32,32)

Stepper motorLeft(
  MOTOR_LEFT_DIR_PIN,
  MOTOR_LEFT_STEP_PIN,
  MOTOR_LEFT_SLEEP_PIN,
  MOTOR_LEFT_RESET_PIN,
  0);


Stepper motorRight(
  MOTOR_RIGHT_DIR_PIN,
  MOTOR_RIGHT_STEP_PIN,
  MOTOR_RIGHT_SLEEP_PIN,
  MOTOR_RIGHT_RESET_PIN,
  1);

SSD_13XX gfx(OLED_CS_PIN, OLED_DC_PIN, OLED_RST_PIN, OLED_MOSI_PIN, OLED_SCLK_PIN);

int test=55;
bool joystickTestMode = false;
bool pidTestMode = false;
bool motorTestMode = false;
float motorTestSpeed = 0;
bool mpuCalibrateMode = false;
unsigned long lastUpdate;

result doAlert(eventMask e, prompt &item);

using namespace Menu;

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

result doToggleMotorCtrl() {
  mgr.enableMotors(robotConfig.motorConfig.enabled);
  return proceed;
}

result doMotorTest() {
  motorTestSpeed = 0;
  return proceed;
}

result doMotorStep() {
  mgr.setStepMode(robotConfig.motorConfig.stepMode);
  return proceed;
}

result doSetPidParams() {
  setPidTunings(robotConfig.pidConfig);
  return proceed;
}

result doToggle() {
  gfx.fillRect(0, 27, 191, 39,GRAY);
  return proceed;
}

/*result doPidTest() {
  gfx.fillRect(0, 27, 191, 39,GRAY);
  return proceed;
}*/

void showJoystickTest() {
  static int lastx=-256, lasty=-256;
  int y = readJoystickX();
  int x = readJoystickY();
  if ((lastx != x) ||
      (lasty != y) ) {
      gfx.fillRect(0, 27, 191, 39,GRAY);
      gfx.setCursor(0,35);
      gfx.setTextColor(WHITE);
      gfx.print("X: ");
      gfx.setTextColor(YELLOW);
      gfx.print(x);

      gfx.setCursor(0,45);
      gfx.setTextColor(WHITE);
      gfx.print("Y: ");
      gfx.setTextColor(YELLOW);
      gfx.print(y);

      gfx.drawCircle(58, 43, 15, WHITE);
      gfx.fillCircle((x>>4) + 58, (-y>>4)+ 43, 5, YELLOW);

      lastx = x;
      lasty = y;
    }
}

void showPidTest()
{
  if (millis() - lastUpdate > 500) {
    lastUpdate = millis();
    gfx.fillRect(0, 27, 97, 37,GRAY);

    gfx.setCursor(0,27);
    gfx.setTextColor(YELLOW);
    gfx.print("Setpoint:  ");
    gfx.setTextColor(RED);
    gfx.println(pidSetpoint);

    gfx.setTextColor(YELLOW);
    gfx.print("Input:  ");
    gfx.setTextColor(GREEN);
    gfx.println(pidInput);

    gfx.setTextColor(YELLOW);
    gfx.print("Output:  ");
    gfx.setTextColor(BLUE);
    gfx.println(pidOutput);
  }
}


void showCalibration() {

  if (millis() - lastUpdate > 2000) {
    lastUpdate = millis();

    gfx.fillRect(0, 27, 96, 37,GRAY);
    gfx.setCursor(0,27);
    gfx.setTextColor(YELLOW);
    gfx.printf("mg: %d, %d, %d\n", mean[GX], mean[GY], mean[GZ]);
    gfx.printf("ma: %d, %d, %d\n", mean[AX], mean[AY], mean[AZ]);
    gfx.printf("og: %d, %d, %d\n", robotConfig.mpuConfig.xGyroOffset,robotConfig.mpuConfig.yGyroOffset,robotConfig.mpuConfig.zGyroOffset);
    gfx.printf("oa: %d, %d, %d\n", robotConfig.mpuConfig.xAccelOffset,robotConfig.mpuConfig.yAccelOffset,robotConfig.mpuConfig.zAccelOffset);
  }
}
result doMpuCalibrate() {

  if (mpuCalibrateMode) {
  }
  gfx.fillRect(0, 27, 191, 39,GRAY);
  return proceed;
}


result doSaveConfig() {
  writeConfig(robotConfig);
  return proceed;
}

result idle(menuOut& o,idleEvent e) {
  if (e==idling) {
    o.println(F("suspended..."));
    o.print(F("time:"));
    o.println(F(millis()));
    o.println(F("press [select]"));
    o.println(F("to continue"));
  }
  return proceed;
}


TOGGLE(joystickTestMode,joystickTest,"Joystick test: ",doNothing,noEvent,noStyle//,doExit,enterEvent,noStyle
  ,VALUE("Off", false, doToggle, enterEvent)
  ,VALUE("On", true, doToggle, enterEvent)
);

TOGGLE(pidTestMode,pidTest,"PID test: ",doNothing,noEvent,noStyle//,doExit,enterEvent,noStyle
  ,VALUE("Off", false, doToggle, enterEvent)
  ,VALUE("On", true, doToggle, enterEvent)
);


TOGGLE(mpuCalibrateMode,mpuCalibrate,"Calibrate: ",doNothing,noEvent,noStyle//,doExit,enterEvent,noStyle
  ,VALUE("Off", false, doMpuCalibrate, enterEvent)
  ,VALUE("On", true, doMpuCalibrate, enterEvent)
);

int selTest=0;
SELECT(selTest,selMenu,"Select Test",doNothing,noEvent,noStyle
  ,VALUE("Zero",0,doNothing,noEvent)
  ,VALUE("One",1,doNothing,noEvent)
  ,VALUE("Two",2,doNothing,noEvent)
);

TOGGLE(robotConfig.motorConfig.enabled,setMotor,"Motor Control: ",doNothing,noEvent,noStyle
  ,VALUE("Off", false, doToggleMotorCtrl, enterEvent)
  ,VALUE("On", true, doToggleMotorCtrl, enterEvent)
);

TOGGLE(motorTestMode,motorTest,"Motor Test: ",doNothing,noEvent,noStyle
  ,VALUE("Off", false, doMotorTest, enterEvent)
  ,VALUE("On", true, doMotorTest, enterEvent)
);


CHOOSE(robotConfig.motorConfig.stepMode,chooseMotorModeMenu,"Motor Mode",doNothing,noEvent,noStyle
  ,VALUE("Full Step", MOTOR_FULL_STEP,doMotorStep,enterEvent)
  ,VALUE("1/2 Step", MOTOR_HALF_STEP,doMotorStep,enterEvent)
  ,VALUE("1/4 Step", MOTOR_QUARTER_STEP,doMotorStep,enterEvent)
  ,VALUE("1/8 Step", MOTOR_EIGHTH_STEP,doMotorStep,enterEvent)
  ,VALUE("1/16 Step", MOTOR_SIXTEENTH_STEP,doMotorStep,enterEvent)
  ,VALUE("1/32 Step", MOTOR_THIRTYSECOND_STEP,doMotorStep,enterEvent)
);

MENU(motorMenu,"Motor Cfg",doNothing,noEvent,noStyle
  ,SUBMENU(setMotor)
  ,SUBMENU(motorTest)
  ,SUBMENU(chooseMotorModeMenu)
  ,EXIT("<Back")
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


TOGGLE(autotune,pidCalibrate,"PID Calibrate: ",doNothing,noEvent,noStyle//,doExit,enterEvent,noStyle
  ,VALUE("Off", false, doToggle, enterEvent)
  ,VALUE("On", true, doToggle, enterEvent)
);


MENU(pidCfgMenu,"PID Cfg",doNothing, noEvent,noStyle
  ,SUBMENU(pidCalibrate)
  ,FIELD(robotConfig.pidConfig.kp,"Kp","",0,200,5,1,doSetPidParams,enterEvent,wrapStyle)
  ,FIELD(robotConfig.pidConfig.ki,"Ki","",0,1500,5,1,doSetPidParams,enterEvent,wrapStyle)
  ,FIELD(robotConfig.pidConfig.kd,"Kd","",0,200,1,0.1,doSetPidParams,enterEvent,wrapStyle)
  ,FIELD(robotConfig.pidAutoTuneConfig.outputStep,"AT output","",0.1,2,0.2,0.1,doSetPidParams,enterEvent,wrapStyle)
  ,FIELD(robotConfig.pidAutoTuneConfig.controlType,"AT Type","",0,1,1,1,doSetPidParams,enterEvent,wrapStyle)
  ,FIELD(robotConfig.pidAutoTuneConfig.lookbackSec,"AT lookback","",0,5,1,1,doSetPidParams,enterEvent,wrapStyle)
  ,FIELD(robotConfig.pidAutoTuneConfig.noiseBand,"AT noise","",0.1,1,0.1,0.05,doSetPidParams,enterEvent,wrapStyle)
  ,EXIT("<Back")
);

MENU(mpuMenu,"MPU Calibrate",doNothing,noEvent,noStyle
  ,SUBMENU(mpuCalibrate)
  ,EXIT("<Back")
);

MENU(pidMenu,"PID test",doNothing,noEvent,noStyle
  ,SUBMENU(pidTest)
  ,EXIT("<Back")
);
MENU(joystickMenu,"Joystick test",doNothing,noEvent,noStyle
  ,SUBMENU(joystickTest)
  ,EXIT("<Back")
);

char* constMEM hexDigit MEMMODE="0123456789ABCDEF";
char* constMEM hexNr[] MEMMODE={"0","x",hexDigit,hexDigit};
char buf1[]="0x11";

MENU(mainMenu,"Robot Control Menu",doNothing,noEvent,wrapStyle
  ,SUBMENU(motorMenu)
  ,SUBMENU(mpuMenu)
  ,SUBMENU(pidCfgMenu)
  ,SUBMENU(pidMenu)
  ,SUBMENU(joystickMenu)
  ,OP("Save Configuration",doSaveConfig,enterEvent)
  ,EXIT("<Exit")
);


// define menu colors --------------------------------------------------------
//  {{disabled normal,disabled selected},{enabled normal,enabled selected, enabled editing}}
//monochromatic color table

const colorDef<uint16_t> colors[] MEMMODE={
  {{BLACK,BLACK},{GRAY,BLUE,BLUE}},//bgColor
  {{GRAY,GRAY},{WHITE,WHITE,WHITE}},//fgColor
  {{WHITE,BLACK},{YELLOW,YELLOW,RED}},//valColor
  {{WHITE,BLACK},{WHITE,YELLOW,YELLOW}},//unitColor
  {{WHITE,GRAY},{GRAY,BLUE,WHITE}},//cursorColor
  {{WHITE,YELLOW},{BLACK,RED,RED}},//titleColor
};

stringIn<4> strIn; //buffer size: 2^5 = 32 bytes, eventually use 0 for a single byte
MENU_INPUTS(in,&strIn);


MENU_OUTPUTS(out,MAX_DEPTH
  ,SSD1331_OUT(gfx,colors,6*MENU_TEXT_SCALE,9*MENU_TEXT_SCALE,{0,0,25,7})
  ,SERIAL_OUT(Serial)
);

NAVROOT(nav,mainMenu,MAX_DEPTH,in,out);

void initMenu() {
  // configure custom options
  options=&myOptions;

  //point a function to be used when menu is suspended
  nav.idleTask=idle;
}



void checkForNavInput(stringIn<4u>& in) {
  int ch = readInput();
  if (ch != -1) {
    in.write(ch);
  }
}

void setup() {
  initInput();
  Serial.begin(115200);

  //delay(5000);

  robotConfig = readConfig();
  initBattery();

  mgr.registerStepper(&motorLeft);
  mgr.registerStepper(&motorRight);

  //motorLeft.setSpeed(0);
  //motorRight.setSpeed(0);

  initMenu();
  gfx.begin(false);
  gfx.setRotation(2);
  gfx.setTextScale(MENU_TEXT_SCALE);
  gfx.setTextWrap(false);
  gfx.fillScreen(BLACK);
  gfx.setTextColor(RED,BLACK);
  gfx.println("Setup ...");

  Serial.flush();
  BLUETOOTH_SERIAL.flush();

  initMpu();

  applyConfig(robotConfig);

  pinMode(LED_PIN, OUTPUT);
  Serial.println("Setup Complete");
}

void loop() {
    checkForNavInput(strIn);
    nav.poll();
    if (joystickTestMode) {
      showJoystickTest();
    }

    if (pidTestMode) {
      showPidTest();
    }

    if (mpuCalibrateMode) {
      if (autoCalibrate(&robotConfig.mpuConfig))
        mpuCalibrateMode = false;
      {
        showCalibration();
      }
    }

    // check battery voltage
    if (!batteryOk()) {
      // disble motor if voltage too low.
      Serial.printf("BATTERY LOW - DISABLING MOTORS");
      robotConfig.motorConfig.enabled = false;
      mgr.enableMotors(robotConfig.motorConfig.enabled);
    }
    delay(5);
}
