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
#include "motor.h"
#include "config.h"
#include "mpu.h"
#include "pid.h"


using namespace Menu;

#define OLED_DC_PIN 26
#define OLED_RST_PIN 255
#define OLED_MOSI_PIN 11
#define OLED_SCLK_PIN 14
#define OLED_CS_PIN 15

#define LED_PIN 13

#define BLUETOOTH_SERIAL Serial5

#define MENU_TEXT_SCALE 1
#define MAX_DEPTH 4
#define GRAY RGB565(32,32,32)

SSD_13XX gfx(OLED_CS_PIN, OLED_DC_PIN, OLED_RST_PIN, OLED_MOSI_PIN, OLED_SCLK_PIN);
_GoBLE goble(&BLUETOOTH_SERIAL);


int test=55;
int ledCount=0;
bool joystickTestMode = false;
bool mpuCalibrateMode = false;

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
  enableMotors(robotConfig.motorConfig.enabled);
  return proceed;
}

result doMotorStep() {
  setMotorStep(robotConfig.motorConfig.stepMode);
  return proceed;
}

result doJoystickTest() {
  gfx.fillRect(0, 27, 191, 39,GRAY);
  return proceed;
}


void showJoystickTest() {
  static int lastx, lasty;
  int y = goble.readJoystickX();
  int x = goble.readJoystickY();
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
      gfx.fillCircle((x>>4) + 50, ((255-y)>>4)+ 35, 5, YELLOW);

      lastx = x;
      lasty = y;
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


TOGGLE(robotConfig.motorConfig.enabled,setMotor,"Motor Control: ",doNothing,noEvent,noStyle//,doExit,enterEvent,noStyle
  ,VALUE("Off", false, doToggleMotorCtrl, enterEvent)
  ,VALUE("On", true, doToggleMotorCtrl, enterEvent)
);


TOGGLE(joystickTestMode,joystickTest,"Joystick test: ",doNothing,noEvent,noStyle//,doExit,enterEvent,noStyle
  ,VALUE("Off", false, doJoystickTest, enterEvent)
  ,VALUE("On", true, doJoystickTest, enterEvent)
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

CHOOSE(robotConfig.motorConfig.stepMode,chooseMotorModeMenu,"Motor Mode",doNothing,noEvent,noStyle
  ,VALUE("Full Step", MOTOR_FULL_STEP,doMotorStep,enterEvent)
  ,VALUE("1/2 Step", MOTOR_HALF_STEP,doMotorStep,enterEvent)
  ,VALUE("1/4 Step", MOTOR_QUARTER_STEP,doMotorStep,enterEvent)
  ,VALUE("1/8 Step", MOTOR_EIGHTH_STEP,doMotorStep,enterEvent)
  ,VALUE("1/16 Step", MOTOR_SIXTEENTH_STEP,doMotorStep,enterEvent)
  ,VALUE("1/32 Step", MOTOR_THIRTYSECOND_STEP,doMotorStep,enterEvent)
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

MENU(motorMenu,"Motor Cfg",doNothing,noEvent,noStyle
  ,SUBMENU(setMotor)
  ,SUBMENU(chooseMotorModeMenu)
  ,EXIT("<Back")
);


MENU(pidMenu,"PID Cfg",doNothing, noEvent,noStyle
  ,FIELD(robotConfig.pidConfig.kp,"Kp","",-1000,1000,10,1,doNothing,enterEvent,wrapStyle)
  ,FIELD(robotConfig.pidConfig.ki,"Ki","",-1000,1000,10,1,doNothing,enterEvent,wrapStyle)
  ,FIELD(robotConfig.pidConfig.kd,"Kd","",-1000,1000,10,1,doNothing,enterEvent,wrapStyle)
  ,EXIT("<Back")
);

MENU(mpuMenu,"MPU Calibrate",doNothing,noEvent,noStyle
  ,SUBMENU(mpuCalibrate)
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
  ,SUBMENU(pidMenu)
  //,SUBMENU(selMenu)
  //,SUBMENU(mpuMenu)
  ,SUBMENU(joystickMenu)
  ,OP("Save Configuration",doSaveConfig,enterEvent)
  //,FIELD(test,"Test","%",0,100,10,1,doNothing,noEvent,wrapStyle)
  //,EDIT("Hex",buf1,hexNr,doNothing,noEvent,noStyle)
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
  //,SSD1331_OUT(gfx,colors,6*MENU_TEXT_SCALE,9*MENU_TEXT_SCALE,{0,0,20,6}, {10,0,10,6})
  //  ,SSD1331_OUT(gfx,colors,6*MENU_TEXT_SCALE,9*MENU_TEXT_SCALE,{0,0,14,8},{14,0,14,8})
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

  int ch = -1;
  if (goble.available()) {
    //Serial.println("input detected");
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

void setup() {
  Serial.begin(115200);
  BLUETOOTH_SERIAL.begin(9600);

  robotConfig = readConfig();
  initMotor();
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


void loop() {
    checkForNavInput(strIn);
    nav.poll();
    if (joystickTestMode) {
      showJoystickTest();
    }

    if (mpuCalibrateMode) {
      if (autoCalibrate(&robotConfig.mpuConfig))
        mpuCalibrateMode = false;
    }
    delay(5);
}
