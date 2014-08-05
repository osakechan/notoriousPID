#include "EEPROMio.h"

void EEPROMRead(unsigned int addr, byte *val, int bytes) {  // read data from the EEPROM
  for (int i = 0; i < bytes; i++) {
    byte temp = EEPROM.read(addr + i);
    *val = temp;
    val++;
  }
}

boolean EEPROMWrite(unsigned int addr, byte *val, int bytes) {  // write data to the EEPROM (only if different from current value)
  boolean change = false;
  for (int i = 0; i < bytes; i++) {
    byte temp = EEPROM.read(addr + i);
    if (temp != *val) {
      EEPROM.write(addr + i, *val);
      change = true;
    }
    val++;
  }
  return change;
}
