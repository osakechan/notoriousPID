#ifndef EEPROMIO_H
#define EEPROMIO_H

#include "Arduino.h"
#include <avr/wdt.h>
#include <SD.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <QueueList.h>

#ifndef DEBUG
#define DEBUG true
#endif

enum {  // byte sizes of data types to make EEPROMRead/Write function calls more readable
  BYTE = 1,
  CHAR = 1,
  INT = 2,
  LONG = 4,
  DOUBLE = 4,
};

struct profileStep {  // struct to encapsulate temperature and duration for fermentation profiles
  double temp;
  double duration;
  profileStep() : temp(0), duration(0) {}
};

extern byte programState;
extern double Setpoint;
extern double Output;
extern double Kp;
extern double Ki;
extern double Kd;
extern double heatKp;
extern double heatKi;
extern double heatKd;
extern double heatOutput;
extern double peakEstimator;
extern File LogFile;
extern File ProFile;
extern LiquidCrystal lcd;
extern const int EEPROM_VER;
extern QueueList <profileStep> profile;
extern void mainUpdate();

void EEPROMReadSettings();
void EEPROMWriteSettings();
void EEPROMWritePresets();

void EEPROMRead(unsigned int addr, byte *val, int bytes);
void EEPROMWrite(unsigned int addr, byte *val, int bytes);

template <typename T> void EEPROMRead(unsigned int addr, T* val, int bytes) {
  byte* temp = (byte*) val;
  EEPROMRead(addr, temp, bytes);
}

template <typename T> void EEPROMWrite(unsigned int addr, T val, int bytes) {
  byte* temp = (byte*) &val;
  EEPROMWrite(addr, temp, bytes);
}

#endif
