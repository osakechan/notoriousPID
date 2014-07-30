// Notorious PID Fermentation Temperature Control v 1.0
#include "Arduino.h"
#include <avr/wdt.h>
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <Wire.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>
#include <OneWire.h>
#include <QueueList.h>
#include <EEPROM.h>
#include "PID_v1.h"
#include "probe.h"
#include "EEPROMio.h"
#include "globals.h"

void setup() {
  pinMode(chipSelect, OUTPUT);             // select pin i/o and enable pullup resistors
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(pushButton, INPUT_PULLUP);
  pinMode(relay1, OUTPUT);                 // configure relay pins and write default HIGH (relay open)
    digitalWrite(relay1, HIGH);
  pinMode(relay2, OUTPUT);
    digitalWrite(relay2, HIGH);
/*pinMode(relay3, OUTPUT);
    digitalWrite(relay3, HIGH);
  pinMode(relay4, OUTPUT);
    digitalWrite(relay4, HIGH);*/
  attachInterrupt(0, encoderChanA, CHANGE);  // interrupt 0 (pin 2) triggered by change
  attachInterrupt(1, encoderChanB, CHANGE);  // interrupt 1 (pin 3) triggered by change
  encoderChanA();  // call interrupt routines once (init rotary encoder state)
  encoderChanB();

  #if DEBUG == true  //start serial at 9600 baud for debuging
    Serial.begin(9600);
  #endif

  Wire.begin();              // initialize rtc communication
  RTC.begin();               // start real time clock
  lcd.createChar(0, delta);  // create custom delta characters for LCD (slots 0-7)
  lcd.createChar(1, rightArrow);
  lcd.createChar(2, disc);
  lcd.createChar(3, circle);
  lcd.createChar(4, dot);
  lcd.createChar(5, inverted);
  lcd.begin(20, 4);          // initialize lcd display
  if (SD.begin(chipSelect, mosi, miso, sck)) lcd.print(F("SDCard Init Success"));  // verify and initialize SD card
    else lcd.print(F("SDCard Failed/Absent"));
  delay(1500);
  SdFile::dateTimeCallback(&dateTime);

  byte ver;
  EEPROMRead(0, &ver, BYTE);  // first byte of EEPROM stores a version # for tracking stored settings
  
  #if DEBUG == true
    Serial.print(F("Current EEPROM ver:"));
    Serial.print(EEPROM_VER);
    Serial.print(F(" value read from EEPROM:"));
    Serial.println(ver);
  #endif
  
  if (ver != EEPROM_VER) {  // EEPROM version number != hard coded version number
    #if DEBUG == true
      Serial.println(F("Outdated or null EEPROM settings. Writing Defaults."));
    #endif
    
    EEPROMWritePresets();  // if version # is outdated, write presets
  }
  EEPROMReadSettings();    // load program settings from EEPROM
  #if DEBUG == true
    Serial.println(F("Settings loaded from EEPROM:"));
  #endif

  fridge.init();
  beer.init();
  
  mainPID.SetTunings(Kp, Ki, Kd);    // set tuning params
  mainPID.SetSampleTime(1000);       // (ms) matches fast sample rate (1 hz)
  mainPID.SetOutputLimits(0.3, 38);  // deg C (~32.5 - ~100 deg F)
  if (programState & 0b100000) mainPID.SetMode(AUTOMATIC);  // set man/auto
    else mainPID.SetMode(MANUAL);
  mainPID.setOutputType(FILTERED);
  mainPID.setFilterConstant(10);
  mainPID.initHistory();

  heatPID.SetTunings(heatKp, heatKi, heatKd);
  heatPID.SetSampleTime(heatWindow);       // sampletime = time proportioning window length
  heatPID.SetOutputLimits(0, heatWindow);  // heatPID output = duty time per window
  if (programState & 0b010000) heatPID.SetMode(AUTOMATIC);
    else heatPID.SetMode(MANUAL);
  heatPID.initHistory();

  encoderPos = 0;  // zero rotary encoder position for main loop
  
  wdt_enable(WDTO_8S);  // enable watchdog timer with 8 second timeout (max setting)
                        // wdt will reset the arduino if there is an infinite loop or other hangup; this is a failsafe device
  #if DEBUG == true
    Serial.print(F("init complete. "));
    Serial.print(millis());
    Serial.print(F("ms elapsed. "));
    Serial.print(freeRAM());
    Serial.println(F(" bytes free SRAM remaining"));
  #endif
}

void loop() {
  wdt_reset();                      // reset the watchdog timer (once timer is set/reset, next reset pulse must be sent before timeout or arduino reset will occur)
  static char listSize = 4;         // for constraining rotary encoder position
  static char lastReportedPos = 1;  // default to 1 to force display initialze on first iteration
  encoderState |= 0b001;            // reset rotary debouncer
  if (encoderPos != lastReportedPos) {
    encoderPos = (encoderPos + listSize) % listSize;  // constrain encoder position
    initDisplay();                  // re-init display on encoder position change
    lastReportedPos = encoderPos;
  }                                                    
  updateDisplay();                       // update display data
  mainUpdate();                            // subroutines manage their own timings, call every loop
  if (!digitalRead(pushButton)) menu();  // call menu routine on rotary button-press
}

void mainUpdate () {                             // call all update subroutines
  probe::startConv();                            // start conversion for all sensors
  if (probe::isReady()) {                        // update sensors when conversion complete
    fridge.update();
    beer.update();
    Input = beer.getFilter();
  }
  if (programState & 0b000100) updateProfile();  // update main Setpoint if fermentation profile active
  mainPID.Compute();                             // update main PID
  updateFridge();                                // update fridge status
  if (programState & 0b000010) writeLog();       // if data capture enabled, run logging routine
}

boolean updateProfile() {             // update temperature profile
  static unsigned int step = 0;
  static unsigned long lastStep = 0;  // last profile step (ms)
  static profileStep Step;            // current profile step
  if ((millis() >= (unsigned long)(lastStep + Step.duration * 3600000UL)) && !profile.isEmpty()) {
    step++;                      // increment step number
    EEPROMWrite(52, step, INT);  // write step number to EEPROM
    Step = profile.pop();        // pop next step off the queue
    Setpoint = Step.temp;        // update Setpoint with new temp (deg C)
    lastStep = millis();
    return true;
  }
  return false;
}

void encoderChanA() {  // interrupt for rotary encoder A channel
  if (encoderState & 0b001) delay (1);  // debounce
  if (digitalRead(encoderPinA) != ((encoderState & 0b100) >> 2)) {    // make sure signal has changed
    encoderState = (encoderState & 0b011) + (~encoderState & 0b100);  // update encoderState with new A value
    if ((encoderState & 0b110) == 4) encoderPos++;  // increment encoder position if A leads B
    encoderState &= 0b110;  // reset debounce flag
  }
}

void encoderChanB() {  // interrupt for rotary encoder B channel
  if (encoderState & 0b001) delay (1);
  if (digitalRead(encoderPinB) != ((encoderState & 0b010) >> 1)) {
    encoderState = (encoderState & 0b101) + (~encoderState & 0b010);
    if ((encoderState & 0b110) == 2) encoderPos--; // decrement encoder position if B leads A
    encoderState &= 0b110;
  }
}

#if DEBUG == true
int freeRAM () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
#endif
