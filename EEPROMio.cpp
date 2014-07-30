#include "EEPROMio.h"

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
  EEPROMRead(38, &peakEstimator, DOUBLE);
  if (programState & 0b000010) {  // load previous logfile if data logging active
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
  if (programState & 0b000100) {  // load previous profile if active
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
      programState |= 0b110100;  //  set PIDs to automatic and enable temperature profile bit
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
  EEPROMWrite(1, (byte)(programState & 0b111110), BYTE);
  EEPROMWrite(2, Setpoint, DOUBLE);
  EEPROMWrite(6, Output, DOUBLE);
  EEPROMWrite(10, Kp, DOUBLE);
  EEPROMWrite(14, Ki, DOUBLE);
  EEPROMWrite(18, Kd, DOUBLE);
  EEPROMWrite(22, heatOutput, DOUBLE);
  EEPROMWrite(26, heatKp, DOUBLE);
  EEPROMWrite(30, heatKi, DOUBLE);
  EEPROMWrite(34, heatKd, DOUBLE);
  EEPROMWrite(38, peakEstimator, DOUBLE);
  if (programState & 0b000010) {  //  write logfile name to EEPROM if data logging active
    EEPROMWrite(42, (byte)LogFile.name()[6], BYTE);
    EEPROMWrite(43, (byte)LogFile.name()[7], BYTE);
  }
}

void EEPROMWritePresets() {  // save defaults to eeprom
  byte temp = EEPROM_VER;
  EEPROMWrite(0, &temp, BYTE);             // update EEPROM version
  EEPROMWrite(1, (byte)0b011000, BYTE);    // default programState (main PID manual, heat PID automatic, deg C, no file operations)
  EEPROMWrite(2, (double)20.00, DOUBLE);   // default main Setpoint
  EEPROMWrite(6, (double)20.00, DOUBLE);   // default main Output for manual operation
  EEPROMWrite(10, (double)10.00, DOUBLE);  // default main Kp
  EEPROMWrite(14, (double)5E-4, DOUBLE);   // default main Ki
  EEPROMWrite(18, (double)00.00, DOUBLE);  // default main Kd
  EEPROMWrite(22, (double)00.00, DOUBLE);  // default HEAT Output for manual operation
  EEPROMWrite(26, (double)05.00, DOUBLE);  // default HEAT Kp
  EEPROMWrite(30, (double)00.25, DOUBLE);  // default HEAT Ki
  EEPROMWrite(34, (double)01.15, DOUBLE);  // default HEAT Kd
  EEPROMWrite(38, (double)05.00, DOUBLE);  // default peakEstimator
}

void EEPROMRead(unsigned int addr, byte *val, int bytes) {  // read data from the EEPROM
  for (int i = 0; i < bytes; i++) {
    byte temp = EEPROM.read(addr + i);
    *val = temp;
    
    #if DEBUG == true
      Serial.print(F("byte:"));
      Serial.print(*val, BIN);
      Serial.print(F(" read from EEPROM &"));
      Serial.println(addr + i);
    #endif
    
    val++;
  }
}

void EEPROMWrite(unsigned int addr, byte *val, int bytes) {  // write data to the EEPROM (only if different from current value)
  for (int i = 0; i < bytes; i++) {
    byte temp = EEPROM.read(addr + i);
    if (temp != *val) {
      EEPROM.write(addr + i, *val);
    
    #if DEBUG == true
      Serial.print(F("byte:"));
      Serial.print(*val, BIN);
      Serial.print(F(" written to EEPROM &"));
      Serial.println(addr + i);
    #endif
    
    }
    val++; 
  }
}
