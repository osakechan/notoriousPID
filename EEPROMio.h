#ifndef EEPROMIO_H
#define EEPROMIO_H

#include "Arduino.h"
#include <EEPROM.h>

enum {  // byte sizes of data types to make EEPROMRead/Write function calls more readable
  BYTE = 1,
  CHAR = 1,
  INT = 2,
  LONG = 4,
  DOUBLE = 4,
};

void EEPROMRead(unsigned int addr, byte *val, int bytes);
boolean EEPROMWrite(unsigned int addr, byte *val, int bytes);

template <typename T> void EEPROMRead(unsigned int addr, T* val, int bytes) {
  byte* temp = (byte*) val;
  EEPROMRead(addr, temp, bytes);
}

template <typename T> boolean EEPROMWrite(unsigned int addr, T val, int bytes) {
  byte* temp = (byte*) &val;
  return EEPROMWrite(addr, temp, bytes);
}

#endif
