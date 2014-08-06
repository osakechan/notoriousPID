#ifndef GLOBALS_H  // ensure this file is included only once
#define GLOBALS_H

const int EEPROM_VER = 11;  // eeprom data tracking

// custom characters for LCD
const byte delta[8] = {
  B00000,
  B00000,
  B00000,
  B00100,
  B01110,
  B11111,
  B11111,
  B00000
};

const uint8_t rightArrow[8] = {
  B11000,
  B10100,
  B10010,
  B10001,
  B10010,
  B10100,
  B11000,
  B00000
};

const byte disc[8] = {
  B01110,
  B11111,
  B11111,
  B11111,
  B01110,
  B00000,
  B00000,
  B00000
};

const byte dot[8] = {
  B00000,
  B00000,
  B00100,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};

const byte circle[8] = {
  B01110,
  B10001,
  B10001,
  B10001,
  B01110,
  B00000,
  B00000,
  B00000
};

const byte inverted[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};

const byte degc[8] = {
  B01000,
  B10100,
  B01000,
  B00011,
  B00100,
  B00100,
  B00011,
  B00000
};
 
const byte degf[8] = {
  B01000,
  B10100,
  B01000,
  B00011,
  B00100,
  B00111,
  B00100,
  B00000
};

struct profileStep {  // struct to encapsulate temperature and duration for fermentation profiles
  double temp;
  double duration;
  profileStep() : temp(0), duration(0) {}
};

// arduino pin declarations:
const byte encoderPinA = 3;   // rotary encoder A channel **interrupt pin**
const byte encoderPinB = 2;   // rotary encoder B channel **interrupt pin**
const byte lcd_d7 = 4;        // lcd D7
const byte lcd_d6 = 5;        // lcd D6
const byte lcd_d5 = 6;        // lcd D5
const byte lcd_d4 = 7;        // lcd D4
const byte lcd_enable = 8;    // lcd enable
const byte lcd_rs = 9;        // lcd RS
const byte chipSelect = 10;   // data logging shield
const byte mosi = 11;         // sd i/o (MOSI)
const byte miso = 12;         // sd i/o (MISO)
const byte sck = 13;          // sd i/o (SCK)
const byte pushButton = A0;   // rotary encoder pushbutton
const byte onewireData = A1;  // one-wire data
const byte relay1 = A2;       // relay 1 (fridge compressor)
const byte relay2 = A3;       // relay 2 (heating element)

volatile char encoderPos;    // a counter for the rotary encoder dial
volatile byte encoderState;  // 3 bit-flag encoder state (A Channel)(B Channel)(is rotating)

OneWire onewire(onewireData);  // declare instance of the OneWire class to communicate with onewire sensors
probe beer(&onewire), fridge(&onewire);

byte programState;  // 6 bit-flag program state -- (mainPID manual/auto)(heatPID manual/auto)(temp C/F)(fermentation profile on/off)(data capture on/off)(file operations) = 0b000000
double Input, Setpoint, Output, Kp, Ki, Kd;  // SP, PV, CO, tuning params for main PID
double heatInput, heatOutput, heatSetpoint, heatKp, heatKi, heatKd;  // SP, PV, CO tuning params for HEAT PID
PID mainPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  // main PID instance for beer temp control (DIRECT: beer temperature ~ fridge(air) temperature)
PID heatPID(&heatInput, &heatOutput, &heatSetpoint, heatKp, heatKi, heatKd, DIRECT);   // create instance of PID class for cascading HEAT control (HEATing is a DIRECT process)

LiquidCrystal lcd(lcd_rs, lcd_enable, lcd_d4, lcd_d5, lcd_d6, lcd_d7);  // declare instance of the LiquidCrystal class for 20x4 LCD
RTC_DS1307 RTC;           // declare instance of Real-time Clock class
File LogFile;             // declare datalogging File object
File ProFile;                      // declare fermentation profile File object
QueueList <profileStep> profile;   // dynamic queue (FIFO) linked list; contains steps for temperature profile

#endif
