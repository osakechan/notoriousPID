// Notorious PID Fermentation Temperature Control v 0.9
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
#include "fridge.h"
#include "globals.h"
#define DEBUG true  // debug flag for including debugging code

void mainUpdate();  // update sensors, PID output, fridge state, write to log, run profiles

boolean updateProfile();  // update temperature profile
void writeLog();          // write new line to log file
void dateTime(uint16_t* date, uint16_t* time);  // date/time callback function for SdFat to timestamp file creation/modification

void initDisplay();    // print static characters to LCD (lables, scrollbar, page names, etc)
void updateDisplay();  // print dynamic characters to LCD (PID/temp values, etc)

void menu();       // change PID and program settings
void mainPIDmode();  // mainPID manual/automatic
void mainPIDsp();    // mainPID setpoint
void heatPIDmode();  // heatPID manual/automatic
void dataLog();      // data logging
void tempProfile();  // temperature profiles
void tempUnit();     // temperature display units C/F
void avrReset();     // restore default settings and reset
void backOut();    // finalize changes and leave menu

void EEPROMReadSettings();   // read saved settings from EEPROM
void EEPROMWriteSettings();  // write current settings to EEPROM
void EEPROMWritePresets();   // write default settings to EEPROM

void encoderChanA();  // manage encoder pin A transitions
void encoderChanB();  // manage encoder pin B transitions

#if DEBUG == true
int freeRAM();  // approximate free SRAM for debugging
#endif

void setup() {
  pinMode(chipSelect, OUTPUT);  // select pin i/o and enable pullup resistors
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(pushButton, INPUT_PULLUP);
  pinMode(relay1, OUTPUT);  // configure relay pins and write default HIGH (relay open)
    digitalWrite(relay1, HIGH);
  pinMode(relay2, OUTPUT);
    digitalWrite(relay2, HIGH);
  attachInterrupt(0, encoderChanA, CHANGE);  // interrupt 0 (pin 2) triggered by change
  attachInterrupt(1, encoderChanB, CHANGE);  // interrupt 1 (pin 3) triggered by change
  encoderPos = 0;
  encoderState = 0b000;
  encoderChanA();  // call interrupt routines once to init rotary encoder
  encoderChanB();

  #if DEBUG == true  //start serial at 9600 baud for debuging
    Serial.begin(9600);
  #endif

  Wire.begin();              // initialize rtc communication
  RTC.begin();               // start real time clock
  lcd.createChar(0, (uint8_t*)delta);  // create custom characters for LCD (slots 0-7)
  lcd.createChar(1, (uint8_t*)rightArrow);
  lcd.createChar(2, (uint8_t*)disc);
  lcd.createChar(3, (uint8_t*)circle);
  lcd.createChar(4, (uint8_t*)dot);
  lcd.createChar(5, (uint8_t*)inverted);
  lcd.createChar(6, (uint8_t*)degc);
  lcd.createChar(7, (uint8_t*)degf);
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
  mainPID.SetSampleTime(1000);       // (ms) matches sample rate (1 hz)
  mainPID.SetOutputLimits(0.3, 38);  // deg C (~32.5 - ~100 deg F)
  if (programState & MAIN_PID_MODE) mainPID.SetMode(AUTOMATIC);  // set man/auto
    else mainPID.SetMode(MANUAL);
  mainPID.setOutputType(FILTERED);
  mainPID.setFilterConstant(10);
  mainPID.initHistory();

  heatPID.SetTunings(heatKp, heatKi, heatKd);
  heatPID.SetSampleTime(heatWindow);       // sampletime = time proportioning window length
  heatPID.SetOutputLimits(0, heatWindow);  // heatPID output = duty time per window
  if (programState & HEAT_PID_MODE) heatPID.SetMode(AUTOMATIC);
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
  encoderState |= DEBOUNCE;         // reset rotary debouncer
  if (encoderPos != lastReportedPos) {
    encoderPos = (encoderPos + listSize) % listSize;  // constrain encoder position
    initDisplay();                  // re-init display on encoder position change
    lastReportedPos = encoderPos;
  }                                                    
  updateDisplay();                       // update display data
  mainUpdate();                            // subroutines manage their own timings, call every loop
  if (!digitalRead(pushButton)) menu();  // call menu routine on rotary button-press
}

void mainUpdate() {                              // call all update subroutines
  probe::startConv();                            // start conversion for all sensors
  if (probe::isReady()) {                        // update sensors when conversion complete
    fridge.update();
    beer.update();
    Input = beer.getFilter();
  }
  if (programState & TEMP_PROFILE) updateProfile();  // update main Setpoint if fermentation profile active
  mainPID.Compute();                             // update main PID
  updateFridge();                                // update fridge status
  if (programState & DATA_LOGGING) writeLog();       // if data capture enabled, run logging routine
}

boolean updateProfile() {
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

void writeLog() {
  static double LogHz = 1;           // datalogging frequency (hz)
  static unsigned long lastLog = 0;  // millis() at last log
  if (millis() >= (unsigned long)(lastLog + 1000/LogHz)) {
    #if DEBUG == true
      Serial.print(F("logging to file... "));
      Serial.print((unsigned long)(millis() - lastLog));
      Serial.print(F("ms elapsed. "));
      Serial.print(lastLog);
      Serial.print(F(" "));
      Serial.print(freeRAM());
      Serial.println(F(" bytes free SRAM remaining"));
    #endif
  
    lastLog = millis();
    DateTime time = RTC.now();
    LogFile.print(lastLog, DEC);
    LogFile.print(F(","));
    LogFile.print(time.year(), DEC);
    LogFile.print(F("/"));
    LogFile.print(time.month(), DEC);
    LogFile.print(F("/"));
    LogFile.print(time.day(), DEC);
    LogFile.print(F(" "));
    LogFile.print(time.hour(), DEC);
    LogFile.print(F(":"));
    LogFile.print(time.minute(), DEC);
    LogFile.print(F(":"));
    LogFile.print(time.second(), DEC);
    LogFile.print(F(","));
    LogFile.print(fridge.getTemp(), DEC);
    LogFile.print(F(","));
    LogFile.print(fridge.getFilter(), DEC);
    LogFile.print(F(","));
    LogFile.print(beer.getTemp(), DEC);
    LogFile.print(F(","));
    LogFile.print(beer.getFilter(), DEC);
    LogFile.print(F(","));
    LogFile.print(Setpoint, DEC);
    LogFile.print(F(","));
    LogFile.print(Output, DEC);
    LogFile.print(F(","));
    LogFile.print(heatSetpoint, DEC);
    LogFile.print(F(","));
    LogFile.print(heatOutput, DEC);
    LogFile.print(F(","));
    LogFile.print(getPeakEstimator());
    LogFile.print(F(","));
    LogFile.println(getFridgeState(0));
    LogFile.flush();
  }
}

void dateTime(uint16_t* date, uint16_t* time) {
  DateTime now = RTC.now();
  *date = FAT_DATE(now.year(), now.month(), now.day());
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

void initDisplay() {
  lcd.clear();
  switch (encoderPos) {
    default:
    case 0:
      lcd.print(F("nPID 1.0"));
      lcd.setCursor(1, 1);
      lcd.print(F("Tf="));
      lcd.setCursor(1, 2);
      lcd.print(F("Tb="));
      lcd.setCursor(11, 1);
      lcd.print(F("SP="));
      lcd.setCursor(5, 3);
      lcd.write((byte)2);
      lcd.write((byte)4);
      lcd.write((byte)4);
      lcd.write((byte)3);
      lcd.write((byte)4);
      lcd.write((byte)4);
      lcd.write((byte)3);
      lcd.write((byte)4);
      lcd.write((byte)4);
      lcd.write((byte)3);
      break;
      
    case 1:
      lcd.print(F("main PID"));
      lcd.setCursor(1, 1);
      lcd.print(F("tP="));
      lcd.setCursor(1, 2);
      lcd.print(F("tD="));
      lcd.setCursor(11, 1);
      lcd.print(F("tI="));
      lcd.setCursor(11, 2);
      lcd.print(F("CO="));
      lcd.setCursor(5, 3);
      lcd.write((byte)3);
      lcd.write((byte)4);
      lcd.write((byte)4);
      lcd.write((byte)2);
      lcd.write((byte)4);
      lcd.write((byte)4);
      lcd.write((byte)3);
      lcd.write((byte)4);
      lcd.write((byte)4);
      lcd.write((byte)3);
      break;

    case 2:
      lcd.print(F("heat PID"));
      lcd.setCursor(1, 1);
      lcd.print(F("tP="));
      lcd.setCursor(1, 2);
      lcd.print(F("tD="));
      lcd.setCursor(11, 1);
      lcd.print(F("tI="));
      lcd.setCursor(11, 2);
      lcd.print(F("CO="));
      lcd.setCursor(5, 3);
      lcd.write((byte)3);
      lcd.write((byte)4);
      lcd.write((byte)4);
      lcd.write((byte)3);
      lcd.write((byte)4);
      lcd.write((byte)4);
      lcd.write((byte)2);
      lcd.write((byte)4);
      lcd.write((byte)4);
      lcd.write((byte)3);
      break;
      
    case 3:
      lcd.print(F(" fridge "));
      lcd.setCursor(1, 2);
      lcd.print(F("pE="));
      lcd.setCursor(11, 2);
      lcd.write((byte)0);
      lcd.print(F("t="));
      lcd.setCursor(5, 3);
      lcd.write((byte)3);
      lcd.write((byte)4);
      lcd.write((byte)4);
      lcd.write((byte)3);
      lcd.write((byte)4);
      lcd.write((byte)4);
      lcd.write((byte)3);
      lcd.write((byte)4);
      lcd.write((byte)4);
      lcd.write((byte)2);
      break;
  }
}

void updateDisplay() {
  lcd.setCursor(9, 0);
  if (programState & DISPLAY_UNIT) lcd.write((byte)7);
    else lcd.write((byte)6);
  if (programState & TEMP_PROFILE) lcd.print(F(" PGM "));
  else {
    if (programState & MAIN_PID_MODE) lcd.print(F(" A "));
      else lcd.print(F(" M "));
    if (programState & HEAT_PID_MODE) lcd.print(F("A "));
      else lcd.print(F("M "));
  }
  if (getFridgeState(0) == IDLE) lcd.print(F("I "));
  if (getFridgeState(0) == HEAT) lcd.print(F("H "));
  if (getFridgeState(0) == COOL) lcd.print(F("C "));
  if (programState & DATA_LOGGING) lcd.print(F("SD"));
    else { lcd.write((byte)5); lcd.write((byte)5); }
  if (!encoderPos) {
    DateTime time = RTC.now();
    lcd.setCursor(11, 2);
    lcd.print((time.hour() - (time.hour() % 10))/10);
    lcd.print(time.hour() % 10);
    lcd.print(F(":"));
    lcd.print(time.minute()/10 % 6);
    lcd.print(time.minute() % 10);
    lcd.print(F(":"));
    lcd.print(time.second()/10 % 6);
    lcd.print(time.second() % 10);
  }
  if (programState & DISPLAY_UNIT) {  // temperature units = deg F
    switch (encoderPos) {         // perform conversion for display
      default:
      case 0:
        lcd.setCursor(4, 1);
        lcd.print(probe::tempCtoF(fridge.getTemp()));
        lcd.setCursor(4, 2);
        lcd.print(probe::tempCtoF(beer.getTemp()));
        lcd.setCursor(14, 1);
        lcd.print(probe::tempCtoF(Setpoint));
        break;

      case 1:
        lcd.setCursor(4, 1);
        lcd.print(mainPID.GetPTerm()*9/5);
        lcd.setCursor(4, 2);
        lcd.print(mainPID.GetDTerm()*9/5);
        lcd.setCursor(14, 1);
        lcd.print(mainPID.GetITerm()*9/5);
        lcd.setCursor(14, 2);
        lcd.print(probe::tempCtoF(Output));
        break;

      case 2:
        lcd.setCursor(4, 1);
        lcd.print(heatPID.GetPTerm()/heatWindow);
        lcd.print('%');
        lcd.setCursor(4, 2);
        lcd.print(heatPID.GetDTerm()/heatWindow);
        lcd.print('%');
        lcd.setCursor(14, 1);
        lcd.print(heatPID.GetITerm()/heatWindow);
        lcd.print('%');
        lcd.setCursor(14, 2);
        lcd.print((double)(heatOutput/heatWindow));
        lcd.print('%');
        break;

      case 3:
        double elapsed = 0;
        lcd.setCursor(0, 1);
        switch (getFridgeState(0)) {
          case IDLE:
            if (getFridgeState(1) == COOL) lcd.print(F("    wait on peak    "));
              else lcd.print(F("       idling       "));
            elapsed = (double)(millis() - getStopTime()) / 60000;   // time since IDLE start in min
            break;

          case COOL:
            lcd.print(F("       cooling      "));
            elapsed = (double)(millis() - getStartTime()) / 60000;  // time since COOL start in min
            break;

          case HEAT:
            elapsed = millis() - getStartTime();  // time since HEAT window start in ms
            if (elapsed < heatOutput) lcd.print(F("      heating      "));
              else lcd.print(F("    idle on heat    "));
            elapsed /= 60000;  // convert ms to min
            break;
        }
        lcd.setCursor(4, 2);
        lcd.print(getPeakEstimator());
        lcd.setCursor(14, 2);
        lcd.print(elapsed);
        break;
    }
  }
  else {                   // temperature units = deg C
    switch (encoderPos) {  // no conversion needed
      default:
      case 0:
        lcd.setCursor(4, 1);
        lcd.print(fridge.getTemp());
        lcd.setCursor(4, 2);
        lcd.print(beer.getTemp());
        lcd.setCursor(14, 1);
        lcd.print(Setpoint);
        break;

      case 1:
        lcd.setCursor(4, 1);
        lcd.print(mainPID.GetPTerm());
        lcd.setCursor(4, 2);
        lcd.print(mainPID.GetDTerm());
        lcd.setCursor(14, 1);
        lcd.print(mainPID.GetITerm());
        lcd.setCursor(14, 2);
        lcd.print(Output);
        break;

      case 2:
        lcd.setCursor(4, 1);
        lcd.print(heatPID.GetPTerm()/heatWindow);
        lcd.print('%');
        lcd.setCursor(4, 2);
        lcd.print(heatPID.GetDTerm()/heatWindow);
        lcd.print('%');
        lcd.setCursor(14, 1);
        lcd.print(heatPID.GetITerm()/heatWindow);
        lcd.print('%');
        lcd.setCursor(14, 2);
        lcd.print((unsigned int)heatOutput/heatWindow);
        lcd.print('%');
        break;
        
      case 3:
        double elapsed = 0;
        lcd.setCursor(0, 1);
        switch (getFridgeState(0)) {
          case IDLE:
            if (getFridgeState(1) == COOL) lcd.print(F("    wait on peak    "));
              else lcd.print(F("       idling       "));
            elapsed = (double)(millis() - getStopTime()) / 60000;   // time since IDLE start in min
            break;

          case COOL:
            lcd.print(F("       cooling      "));
            elapsed = (double)(millis() - getStartTime()) / 60000;  // time since COOL start in min
            break;

          case HEAT:
            elapsed = millis() - getStartTime();  // time since HEAT window start in ms
            if (elapsed < heatOutput) lcd.print(F("      heating      "));
              else lcd.print(F("    idle on heat    "));
            elapsed /= 60000;  // convert ms to min
            break;
        }
        lcd.setCursor(4, 2);
        lcd.print(getPeakEstimator());
        lcd.setCursor(14, 2);
        lcd.print(elapsed);
        break;
    }
  }
}

void menu() {
  #if DEBUG == true
    Serial.print(F("control suspended, entering menu... "));
    Serial.print(freeRAM());
    Serial.println(F(" bytes free SRAM remaining"));
  #endif

  static char menu_list [8][21] = {"Main PID: Mode", "Main PID: SP", "Heat PID: Mode", "[SD] Logging", "[SD] Profiles", "Display Units", "Restore & Reset", "BACK"};
  boolean exit = false;
  encoderPos = 0;
  do {wdt_reset(); mainUpdate();} while (!digitalRead(pushButton));  // wait for user to let go of button; continue to poll wdt with reset pulse
  
  do {  // main menu loop; display current menu option
    wdt_reset();
    mainUpdate();
    static char lastReportedPos = 1;
    static char listSize = 8;
    encoderState |= DEBOUNCE;
    if (lastReportedPos != encoderPos) {
      encoderPos = (encoderPos + listSize) % listSize;
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print(menu_list[(encoderPos - 1 + listSize) % listSize]);
      lcd.setCursor(0, 1);
      lcd.write((byte)1);
      lcd.print(menu_list[encoderPos]);
      lcd.setCursor(1, 2);
      lcd.print(menu_list[(encoderPos + 1 + listSize) % listSize]);
      lcd.setCursor(1, 3);
      lcd.print(menu_list[(encoderPos + 2 + listSize) % listSize]);
      lastReportedPos = encoderPos;
    }
    if (!digitalRead(pushButton)) {
      switch (encoderPos) {  // main menu switch; run subroutine for current menu option on rotary encoder push

        case 0:  // main PID operation -- manual / automatic
          do {wdt_reset(); mainUpdate();} while (!digitalRead(pushButton));
          mainPIDmode();
          encoderPos = 0;  // return encoder position to current option on the menu
          break;

        case 1:  // main PID Setpoint
          do {wdt_reset(); mainUpdate();} while (!digitalRead(pushButton));
          mainPIDsp();
          encoderPos = 1;
          break;

        case 2:  // heat PID operation -- manual / automatic
          do {wdt_reset(); mainUpdate();} while (!digitalRead(pushButton));
          heatPIDmode();
          encoderPos = 2;
          break;
          
        case 3:  // data logging operations
          do {wdt_reset(); mainUpdate();} while (!digitalRead(pushButton));
          dataLog();
          encoderPos = 3;
          break;

        case 4:  // fermentation temperature profiles
          do {wdt_reset(); mainUpdate();} while (!digitalRead(pushButton));
          tempProfile(); 
          encoderPos = 4;
          break;

        case 5:  // temperature units C/F
          do {wdt_reset(); mainUpdate();} while (!digitalRead(pushButton));
          tempUnit();          
          encoderPos = 5;
          break;
        
        case 6:  // reset all settings to defaults and reboot arduino
          do {wdt_reset(); mainUpdate();} while (!digitalRead(pushButton));
          avrReset();
          encoderPos = 6;
          break;
          
        case 7:  // backOut of the menu and save settings; do file operations if needed
          backOut();
          exit = true;  // exit the menu loop
          break;
      }
      do {wdt_reset(); mainUpdate();} while (!digitalRead(pushButton));
      lastReportedPos += 1;  // force re-draw of menu list
    }
  } while (!exit);
  encoderPos = 0;  // zero encoder position for main display
  initDisplay();   // re-initialize main display
}

void mainPIDmode() {
  if (programState & TEMP_PROFILE) {
    lcd.setCursor(0, 2);
    lcd.print(F(" PROFILE IS RUNNING "));
    unsigned long start = millis();
    do { wdt_reset(); mainUpdate(); } while (millis() <= (unsigned long)(start + 1500));  // wait for 1.5 seconds without hard delay
    return;
  }
  char listSize = 2;
  encoderPos = (programState & MAIN_PID_MODE) >> 5;
  char lastReportedPos = encoderPos + 1;
  lcd.setCursor(0, 2);
  lcd.print(F("                    "));
  lcd.setCursor(2, 2);
  lcd.write((byte)1);
  do {
    wdt_reset();
    mainUpdate();
    encoderState |= DEBOUNCE;
    if (lastReportedPos != encoderPos) {
      encoderPos = (encoderPos + listSize) % listSize;
      lcd.setCursor(3, 2);
      if (encoderPos) lcd.print(F("Automatic"));
        else lcd.print(F("Manual   "));
      lastReportedPos = encoderPos;
    }
  } while (digitalRead(pushButton));
          
  #if DEBUG == true
    if (encoderPos) Serial.print(F("main PID set to automatic. "));
    else Serial.print(F("main PID set to manual. "));
    Serial.print(freeRAM());
    Serial.println(F(" bytes free SRAM remaining"));
  #endif

  if (encoderPos) {programState |= MAIN_PID_MODE;}  // set main PID to automatic mode
  else {  // set main PID to manual mode; user entry of main Output for manual only
    programState &= ~MAIN_PID_MODE;
    do {wdt_reset(); mainUpdate();} while (!digitalRead(pushButton));
    char listSize = 100;
    if (programState & DISPLAY_UNIT) Output = probe::tempCtoF(Output);  // if display unit = deg F, convert Output
    encoderPos = int(Output);
    char lastReportedPos = encoderPos + 1;
    lcd.setCursor(3, 2);
    lcd.print(F("                 "));
    do {  // coarse-grained ajustment (integers)
      wdt_reset();
      mainUpdate();
      encoderState |= DEBOUNCE;
      if (lastReportedPos != encoderPos) {
        encoderPos = (encoderPos + listSize) % listSize;
        lcd.setCursor(3, 2);
        lcd.print(encoderPos + Output - int(Output));
        if (programState & DISPLAY_UNIT) lcd.print(F(" \337F"));
          else lcd.print(F(" \337C"));
        lastReportedPos = encoderPos;
      }
      lcd.setCursor(4, 2);
      lcd.cursor();
    } while (digitalRead(pushButton));
    lcd.noCursor();
    Output = encoderPos + Output - int(Output);
    encoderPos = (Output - int(Output)) * 10;
    lastReportedPos = encoderPos + 1;
    Output = int(Output);
    listSize = 10;
    do {wdt_reset(); mainUpdate();} while (!digitalRead(pushButton));
    do {  // fine-grained ajustment (tenths)
      wdt_reset();
      mainUpdate();
      encoderState |= DEBOUNCE;
      if (lastReportedPos != encoderPos) {
        encoderPos = (encoderPos + listSize) % listSize;
        lcd.setCursor(3, 2);
        lcd.print(Output + double(encoderPos)/10);
        if (programState & DISPLAY_UNIT) lcd.print(F(" \337F"));
          else lcd.print(F(" \337C"));
        lastReportedPos = encoderPos;
      }
      lcd.setCursor(6, 2);
      lcd.cursor();
    } while (digitalRead(pushButton));
    lcd.noCursor();
    Output = Output + double(encoderPos)/10; 
    if (programState & DISPLAY_UNIT) Output = probe::tempFtoC(Output);  // if display is in deg F, convert user entry back to native deg C
    Output = constrain(Output, 0.3, 38);  // constrain main PID Output to allowed range 0.3 - 38 deg C (~32.5 - ~100 deg F)
  }
            
  #if DEBUG == true
    Serial.print(F("main PID Output set to:"));
    Serial.print(Output);
    Serial.print(F(" "));
    Serial.print(freeRAM());
    Serial.println(F(" bytes free SRAM remaining"));
  #endif
}

void mainPIDsp() {
  if (programState & TEMP_PROFILE) {
    lcd.setCursor(0, 2);
    lcd.print(F(" PROFILE IS RUNNING "));
    unsigned long start = millis();
    do { wdt_reset(); mainUpdate(); } while (millis() <= (unsigned long)(start + 1500));
    return;
  }
  lcd.setCursor(0, 2);
  lcd.print(F("                    "));
  lcd.setCursor(2, 2);
  lcd.write((byte)1);
  char listSize = 100;
  if (programState & DISPLAY_UNIT) Setpoint = probe::tempCtoF(Setpoint);
  encoderPos = int(Setpoint);
  char lastReportedPos = encoderPos + 1;
  do {  // coarse-grained ajustment (integers)
    wdt_reset();
    mainUpdate();
    encoderState |= DEBOUNCE;
    if (lastReportedPos != encoderPos) {
      encoderPos = (encoderPos + listSize) % listSize;
      lcd.setCursor(3, 2);
      lcd.print(encoderPos + Setpoint - int(Setpoint));
      if (programState & DISPLAY_UNIT) lcd.print(F(" \337F"));
        else lcd.print(F(" \337C"));
      lastReportedPos = encoderPos;
    }
    lcd.setCursor(4, 2);
    lcd.cursor();
  } while (digitalRead(pushButton));
  lcd.noCursor();
  Setpoint = encoderPos + Setpoint - int(Setpoint);

  encoderPos = (Setpoint - int(Setpoint)) * 10;
  lastReportedPos = encoderPos + 1;
  Setpoint = int(Setpoint);
  listSize = 10;
  do {wdt_reset(); mainUpdate();} while (!digitalRead(pushButton));
  do {  // fine-grained ajustment (tenths)
    wdt_reset();
    mainUpdate();
    encoderState |= DEBOUNCE;
    if (lastReportedPos != encoderPos) {
      encoderPos = (encoderPos + listSize) % listSize;
      lcd.setCursor(3, 2);
      lcd.print(Setpoint + double(encoderPos)/10);
      if (programState & DISPLAY_UNIT) lcd.print(F(" \337F"));
        else lcd.print(F(" \337C"));
      lastReportedPos = encoderPos;
    }
    lcd.setCursor(6, 2);
    lcd.cursor();
  } while (digitalRead(pushButton));
  lcd.noCursor();
  Setpoint = Setpoint + double(encoderPos)/10;
  if (programState & DISPLAY_UNIT) Setpoint = probe::tempFtoC(Setpoint);

  #if DEBUG == true
    Serial.print(F("main PID Setpoint set to:"));
    Serial.print(Setpoint);
    Serial.print(F(" "));
    Serial.print(freeRAM());
    Serial.println(F(" bytes free SRAM remaining"));
  #endif
}

void heatPIDmode() {
  if (programState & TEMP_PROFILE) {
    lcd.setCursor(0, 2);
    lcd.print(F(" PROFILE IS RUNNING "));
    unsigned long start = millis();
    do { wdt_reset(); mainUpdate(); } while (millis() <= (unsigned long)(start + 1500));
    return;
  }
  char listSize = 2;
  encoderPos = (programState & HEAT_PID_MODE) >> 4;
  char lastReportedPos = encoderPos + 1;
  lcd.setCursor(0, 2);
  lcd.print(F("                    "));
  lcd.setCursor(2, 2);
  lcd.write((byte)1);
  do {
    wdt_reset();
    mainUpdate();
    encoderState |= DEBOUNCE;
    if (lastReportedPos != encoderPos) {
      encoderPos = (encoderPos + listSize) % listSize;
      lcd.setCursor(3, 2);
      if (encoderPos) lcd.print(F("Automatic"));
        else lcd.print(F("Manual   "));
      lastReportedPos = encoderPos;
    }
  } while (digitalRead(pushButton));

  #if DEBUG == true
    if (encoderPos) Serial.print(F("heat PID set to Automatic. "));
      else Serial.print(F("heat PID set to Manual. "));
    Serial.print(freeRAM());
    Serial.println(F(" bytes free SRAM remaining"));
  #endif

  if (encoderPos) programState |= HEAT_PID_MODE;  // set heat PID to automatic mode
  else {  // set heat PID to manual mode; user entry of heat Output for manual only
    programState &= ~HEAT_PID_MODE;
    do {wdt_reset(); mainUpdate();} while (!digitalRead(pushButton));
    char listSize = 100;
    if (programState & DISPLAY_UNIT) heatSetpoint = probe::tempCtoF(heatSetpoint);
    encoderPos = int(heatSetpoint);
    char lastReportedPos = encoderPos + 1;
    lcd.setCursor(3, 2);
    lcd.print(F("                 "));
    do {  // coarse-grained ajustment (integers)
      wdt_reset();
      mainUpdate();
      encoderState |= DEBOUNCE;
      if (lastReportedPos != encoderPos) {
        encoderPos = (encoderPos + listSize) % listSize;
        lcd.setCursor(3, 2);
        lcd.print(encoderPos + heatSetpoint - int(heatSetpoint));
        if (programState & DISPLAY_UNIT) lcd.print(F(" \337F"));
          else lcd.print(F(" \337C"));
        lastReportedPos = encoderPos;
      }
      lcd.setCursor(4, 2);
      lcd.cursor();
    } while (digitalRead(pushButton));
    lcd.noCursor();
    heatSetpoint = encoderPos + heatSetpoint - int(heatSetpoint);

    encoderPos = (heatSetpoint - int(heatSetpoint)) * 10;
    lastReportedPos = encoderPos + 1;
    heatSetpoint = int(heatSetpoint);
    listSize = 10;
    do {wdt_reset(); mainUpdate();} while (!digitalRead(pushButton));
    do {  // fine-grained ajustment (tenths)
      wdt_reset();
      mainUpdate();
      encoderState |= DEBOUNCE;
      if (lastReportedPos != encoderPos) {
        encoderPos = (encoderPos + listSize) % listSize;
        lcd.setCursor(3, 2);
        lcd.print(heatSetpoint + double(encoderPos)/10);
        if (programState & DISPLAY_UNIT) lcd.print(F(" \337F"));
          else lcd.print(F(" \337C"));
        lastReportedPos = encoderPos;
      }
      lcd.setCursor(6, 2);
      lcd.cursor();
    } while (digitalRead(pushButton));
    lcd.noCursor();
    heatSetpoint = heatSetpoint + double(encoderPos)/10; 
    if (programState & DISPLAY_UNIT) heatSetpoint = probe::tempFtoC(heatSetpoint);
  }

  #if DEBUG == true
    Serial.print(F("heat PID Setpoint set to:"));
    Serial.print(heatSetpoint);
    Serial.print(F(" "));
    Serial.print(freeRAM());
    Serial.println(F(" bytes free SRAM remaining"));
  #endif
}

void dataLog() {
  char listSize = 2;
  encoderPos = (programState & DATA_LOGGING) >> 1;
  char lastReportedPos = encoderPos + 1;
  lcd.setCursor(0, 2);
  lcd.print(F("                    "));
  lcd.setCursor(2, 2);
  lcd.write((byte)1);
  do {
    wdt_reset();
    mainUpdate();
    encoderState |= DEBOUNCE;
    if (lastReportedPos != encoderPos) {
      encoderPos = (encoderPos + listSize) % listSize;
      lcd.setCursor(3, 2);
      if (encoderPos) lcd.print(F("Enabled "));
        else lcd.print(F("Disabled"));
      lastReportedPos = encoderPos;
    }
  } while (digitalRead(pushButton));
  if (encoderPos) {
    if (!(programState & (DATA_LOGGING + FILE_OPS))) {
      #if DEBUG == true
        Serial.print(F("New logfile pending... "));
        Serial.print(freeRAM());
        Serial.println(F(" bytes free SRAM remaining"));
      #endif

      programState += DATA_LOGGING + FILE_OPS;  // start new LogFile on menu exit
    }
    else if ((programState & (DATA_LOGGING + FILE_OPS)) == FILE_OPS) {
      #if DEBUG == true
        Serial.print(F("Pending close operation canceled... "));
        Serial.print(freeRAM());
        Serial.println(F(" bytes free SRAM remaining"));
      #endif
                
      programState = programState & ~(DATA_LOGGING + FILE_OPS) + DATA_LOGGING;  // cancel pending file close and leave log running
    }
  }
  else {
    if ((programState & (DATA_LOGGING + FILE_OPS)) == DATA_LOGGING) {
      #if DEBUG == true
        Serial.print(F("Logfile close pending... "));
        Serial.print(freeRAM());
        Serial.println(F(" bytes free SRAM remaining"));
      #endif
              
      programState = (programState & ~(DATA_LOGGING + FILE_OPS)) + FILE_OPS;  // close current LogFile on menu exit
    }
    else if ((programState & (DATA_LOGGING + FILE_OPS)) == DATA_LOGGING + FILE_OPS) {
      #if DEBUG == true
        Serial.print(F("Pending new operation canceled... "));
        Serial.print(freeRAM());
        Serial.println(F(" bytes free SRAM remaining"));
      #endif

      programState &= ~(DATA_LOGGING + FILE_OPS);  // cancel pending file opening
    }
  }
}

void tempProfile() {              // manage SP profiles
  if (programState & TEMP_PROFILE) {  // if profile already running
    char listSize = 2;
    encoderPos = 0;
    char lastReportedPos = 1;
    lcd.setCursor(0, 2);
    lcd.print(F(" STOP PROFILE?      "));
    lcd.setCursor(15, 2);
    lcd.write((byte)1);
    do {
      wdt_reset();
      mainUpdate();
      encoderState |= DEBOUNCE;
      if (lastReportedPos != encoderPos) {
        encoderPos = (encoderPos + listSize) % listSize;
        lcd.setCursor(16, 2);
        if (encoderPos) lcd.print(F("YES"));
          else lcd.print(F("NO "));
        lastReportedPos = encoderPos;
      }
    } while (digitalRead(pushButton));
    if (encoderPos) {  // empty profile queue, reset program flag and return to main menu
      programState &= ~TEMP_PROFILE;
      while (!profile.isEmpty()) {
        wdt_reset();
        mainUpdate();
        profile.pop();
      }
    }
    encoderPos = 4;
    return;
  }
  encoderPos = 0;
  char lastReportedPos = 1;
  File root = SD.open("/PROFILES/", FILE_READ);  //  open root to profile directory
  if (!root) {  // profile directory does not exist
    lcd.setCursor(0, 2);
    lcd.print(F(" /PROFILES/ MISSING "));
    unsigned long start = millis();
    do { wdt_reset(); mainUpdate(); } while (millis() <= (unsigned long)(start + 1500));
    return;
  }
  lcd.setCursor(0, 2);
  lcd.print(F("                    "));
  do {
    wdt_reset();
    mainUpdate();
    encoderState |= DEBOUNCE;
    if (lastReportedPos != encoderPos) {
      do {
        ProFile = root.openNextFile();  // open next file in /profiles/
      } while (ProFile.isDirectory());  // ignore directories
      if (!ProFile) {                   // no more files in /profiles/
        root.rewindDirectory();         // return to top of /profiles/
        lcd.setCursor(2, 2);
        lcd.write((byte)1);
        lcd.print(F("BACK        "));
      } 
      else {
        lcd.setCursor(2, 2);
        lcd.write((byte)1);
        lcd.print(ProFile.name());  // print current filename to LCD
      }
      lastReportedPos = encoderPos;
    }
  } while (digitalRead(pushButton));
  do {wdt_reset(); mainUpdate();} while (!digitalRead(pushButton));
  if (ProFile) {
    char filename[12];
    for (int i = 0; i++; i < 8) {  // store profile name in EEPROM
      EEPROMWrite(44 + i, ProFile.name()[i], BYTE);
    }
    char buff[20];     // char buffer for file data
    profileStep Step;  // temporary profile step to push to queue
    while (ProFile.peek() != -1) {  // if not EOF, read temperature,duration one byte at a time
      wdt_reset();
      mainUpdate();
      for (int i = 0; i < 20; i++) {  // read step temperature (deg C) into buffer until ',' found
        buff[i] = ProFile.read();
        if (buff[i] == ',') break;
      }
      Step.temp = strtod(buff, 0);    // convert (char)buffer to double
      memset(buff, 0, 20);            // reset buffer
      for (int i = 0; i < 20; i++) {  // read step duration (hours) into buffer until newline found
        buff[i] = ProFile.read();
        if (buff[i] == '\n') break;
      }
      Step.duration = strtod(buff, 0);
      if (Step.temp || Step.duration) profile.push(Step);  // push (non-null) fermentation profile step into queue
    }
    programState |= MAIN_PID_MODE + HEAT_PID_MODE + TEMP_PROFILE;  //  set PIDs to automatic and enable temperature profile bit
  }
  root.close();
  ProFile.close();
}

void tempUnit() {
  char listSize = 2;
  encoderPos = (programState & DISPLAY_UNIT) >> 3;
  char lastReportedPos = encoderPos + 1;
  lcd.setCursor(0, 2);
  lcd.print(F("                    "));
  lcd.setCursor(2, 2);
  lcd.write((byte)1);
  lcd.print(F(" \337"));
  do {
    wdt_reset();
    mainUpdate();
    encoderState |= DEBOUNCE;
    if (lastReportedPos != encoderPos) {
      encoderPos = (encoderPos + listSize) % listSize;
      lcd.setCursor(5, 2);
      if (encoderPos) lcd.print(F("F"));
        else lcd.print(F("C"));
      lastReportedPos = encoderPos;
    }
  } while (digitalRead(pushButton));
  if (encoderPos) programState |= DISPLAY_UNIT;
    else programState &= ~DISPLAY_UNIT;

  #if DEBUG == true
    if (encoderPos) Serial.print(F("Units set to deg F."));
      else Serial.print(F("Units set to deg C."));
    Serial.print(freeRAM());
    Serial.println(F(" bytes free SRAM remaining"));
  #endif
}

void avrReset() {
  char listSize = 2;
  encoderPos = 0;
  char lastReportedPos = 1;
  lcd.setCursor(0, 2);
  lcd.print(F("  SURE?             "));
  lcd.setCursor(8, 2);
  lcd.write((byte)1);
  do {
    wdt_reset();
    mainUpdate();
    encoderState |= DEBOUNCE;
    if (lastReportedPos != encoderPos) {
      encoderPos = (encoderPos + listSize) % listSize;
      lcd.setCursor(9, 2);
      if (encoderPos) lcd.print(F("YES"));
        else lcd.print(F("NO "));
      lastReportedPos = encoderPos;
    }
  } while (digitalRead(pushButton));
  if (encoderPos) {
    #if DEBUG == true
      Serial.println(F("Restoring PID to default settings and rebooting..."));
    #endif

    EEPROMWritePresets();    // restore default settings to EEPROM
    wdt_enable(WDTO_250MS);  // change watchdog timer to 250ms
    do {} while (true);      // infinte loop; wait for arduino to reset after 250ms timeout with no watchdog reset pulse
  }
}

void backOut() {  //  finalize any changes in preparation for menu exit
  if (programState & MAIN_PID_MODE) mainPID.SetMode(AUTOMATIC);
    else mainPID.SetMode(MANUAL);
  if (programState & HEAT_PID_MODE) heatPID.SetMode(AUTOMATIC);
    else heatPID.SetMode(MANUAL);
  if ((programState & (DATA_LOGGING + FILE_OPS)) == DATA_LOGGING + FILE_OPS) {  // create a new comma seperated value LogFile
    char filename[] = "LOGGER00.CSV";
    for (int i = 0; i < 100; i++) {
      filename[6] = i/10 + '0';
      filename[7] = i%10 + '0';
      if (!SD.exists(filename)) {  // only open a new file if it doesn't exist
        LogFile = SD.open(filename, FILE_WRITE);
        if (LogFile) {
          #if DEBUG == true
            Serial.print(F("New file success:"));
            Serial.print(filename);
            Serial.print(F(" "));
            Serial.print(freeRAM());
            Serial.println(F(" bytes free SRAM remaining"));
          #endif
        }
        break;
      }
    }
    LogFile.println(F("millis,datetime,fridge actual,fridge filter,beer actual,beer filter,mainSP,mainCO,heatSP,heatCO,peak estimator,fridge state"));
    LogFile.flush();  //print header to file
  }
  if ((programState & (DATA_LOGGING + FILE_OPS)) == FILE_OPS) {
    #if DEBUG == true
      Serial.print(F("Logfile closed. "));
      Serial.print(freeRAM());
      Serial.println(F(" bytes free SRAM remaining"));
    #endif

    LogFile.close();  // close LogFile
  }
  programState &= ~FILE_OPS;  // reset file change flag
  EEPROMWriteSettings();     // update settings stored in non-volatile memory
}

void EEPROMReadSettings() {  // read settings from EEPROM
  EEPROMRead(1, &programState, BYTE);
  EEPROMRead(2, &Setpoint, DOUBLE);
  EEPROMRead(6, &Output, DOUBLE);
  EEPROMRead(10, &Kp, DOUBLE);
  EEPROMRead(14, &Ki, DOUBLE);
  EEPROMRead(18, &Kd, DOUBLE);
  EEPROMRead(22, &heatOutput, DOUBLE);
  EEPROMRead(26, &heatKp, DOUBLE);
  EEPROMRead(30, &heatKi, DOUBLE);
  EEPROMRead(34, &heatKd, DOUBLE);
  double* estimator = getPeakEstimatorAddr();
  EEPROMRead(38, &estimator, DOUBLE);
  if (programState & DATA_LOGGING) {  // load previous logfile if data logging active
    char filename[] = "LOGGER00.CSV";
    EEPROMRead(42, &filename[6], BYTE); 
    EEPROMRead(43, &filename[7], BYTE);
    if (SD.exists(filename)) {
      LogFile = SD.open(filename, FILE_WRITE);
      lcd.clear();
      lcd.print(filename);
      lcd.print(F(" open."));
      #if DEBUG == true
        Serial.print(filename);
        Serial.println(F(" re-opened."));
      #endif
      delay(1500);
    }
  }
  if (programState & TEMP_PROFILE) {  // load previous profile if active
    char filename[] = "/PROFILES/        .PGM";
    for (int i = 0; i++; i < 8) {
      EEPROMRead(44 + i, &filename[10 + i], BYTE);
    }
    #if DEBUG == true
      Serial.print(F("Opening file:"));
      Serial.println(filename);
    #endif
    ProFile = SD.open(filename, FILE_READ);
    if (ProFile) {
      lcd.clear();
      lcd.print(filename);
      lcd.print(F(" open."));
      #if DEBUG == true
        Serial.print(filename);
        Serial.println(F(" re-opened."));
      #endif
      delay(1500);
      char buff[20];     // char buffer for file data
      profileStep Step;  // temporary profile step to push to queue
      while (ProFile.peek() != -1) {  // if not EOF, read temperature,duration one byte at a time
        wdt_reset();
        mainUpdate();
        for (int i = 0; i < 20; i++) {  // read step temperature (deg C) into buffer until ',' found
          buff[i] = ProFile.read();
          if (buff[i] == ',') break;
        }
        Step.temp = strtod(buff, 0);    // convert (char)buffer to double
        memset(buff, 0, 20);            // reset buffer
        for (int i = 0; i < 20; i++) {  // read step duration (hours) into buffer until newline found
          buff[i] = ProFile.read();
          if (buff[i] == '\n') break;
        }
        Step.duration = strtod(buff, 0);
        if (Step.temp || Step.duration) profile.push(Step);  // push (non-null) fermentation profile step into queue
      }
      programState |= MAIN_PID_MODE + HEAT_PID_MODE + TEMP_PROFILE;  //  set PIDs to automatic and enable temperature profile bit
    }
    ProFile.close();
    unsigned int step = 0;
    unsigned int count = 1;
    EEPROMRead(52, &step, INT);
    while (step > count) {  // reset queue to last step
      count++;
      wdt_reset();
      mainUpdate();
      profile.pop();
    }
  }
}

void EEPROMWriteSettings() {  // write current settings to EEPROM
  byte temp = EEPROM_VER;
  EEPROMWrite(0, &temp, BYTE);
  EEPROMWrite(1, (byte)(programState & ~FILE_OPS), BYTE);
  EEPROMWrite(2, Setpoint, DOUBLE);
  EEPROMWrite(6, Output, DOUBLE);
  EEPROMWrite(10, Kp, DOUBLE);
  EEPROMWrite(14, Ki, DOUBLE);
  EEPROMWrite(18, Kd, DOUBLE);
  EEPROMWrite(22, heatOutput, DOUBLE);
  EEPROMWrite(26, heatKp, DOUBLE);
  EEPROMWrite(30, heatKi, DOUBLE);
  EEPROMWrite(34, heatKd, DOUBLE);
  EEPROMWrite(38, getPeakEstimator(), DOUBLE);
  if (programState & DATA_LOGGING) {  //  write logfile name to EEPROM if data logging active
    EEPROMWrite(42, (byte)LogFile.name()[6], BYTE);
    EEPROMWrite(43, (byte)LogFile.name()[7], BYTE);
  }
}

void EEPROMWritePresets() {      // save defaults to eeprom
  byte temp = EEPROM_VER;
  EEPROMWrite(0, &temp, BYTE);                 // update EEPROM version
  EEPROMWrite(1, (byte)DISPLAY_UNIT, BYTE);    // default programState (main PID manual, heat PID manual, deg F, no file operations)
  EEPROMWrite(2, (double)20.00, DOUBLE);       // default main Setpoint
  EEPROMWrite(6, (double)20.00, DOUBLE);       // default main Output for manual operation
  EEPROMWrite(10, (double)10.00, DOUBLE);      // default main Kp
  EEPROMWrite(14, (double)5E-4, DOUBLE);       // default main Ki
  EEPROMWrite(18, (double)500.0, DOUBLE);     // default main Kd
  EEPROMWrite(22, (double)00.00, DOUBLE);      // default HEAT Output for manual operation
  EEPROMWrite(26, (double)05.00, DOUBLE);      // default HEAT Kp
  EEPROMWrite(30, (double)00.25, DOUBLE);      // default HEAT Ki
  EEPROMWrite(34, (double)01.15, DOUBLE);      // default HEAT Kd
  EEPROMWrite(38, (double)05.00, DOUBLE);      // default peakEstimator
}

void encoderChanA() {  // interrupt for rotary encoder A channel
  if (encoderState & DEBOUNCE) delay (1);  // debounce
  if (digitalRead(encoderPinA) != ((encoderState & CHAN_A) >> 2)) {    // make sure signal has changed
    encoderState = (encoderState & ~CHAN_A) + (~encoderState & CHAN_A);  // update encoderState with new A value
    if ((encoderState & ~DEBOUNCE) == CHAN_A) encoderPos++;  // increment encoder position if A leads B
    encoderState &= ~DEBOUNCE;  // reset debounce flag
  }
}

void encoderChanB() {  // interrupt for rotary encoder B channel
  if (encoderState & DEBOUNCE) delay (1);
  if (digitalRead(encoderPinB) != ((encoderState & CHAN_B) >> 1)) {
    encoderState = (encoderState & ~CHAN_B) + (~encoderState & CHAN_B);
    if ((encoderState & ~DEBOUNCE) == CHAN_B) encoderPos--; // decrement encoder position if B leads A
    encoderState &= ~DEBOUNCE;
  }
}

#if DEBUG == true
int freeRAM() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
#endif
