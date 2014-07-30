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
    LogFile.print(peakEstimator);
    LogFile.print(F(","));
    LogFile.println(fridgeState[0]);
    LogFile.flush();
  }
}

void dateTime(uint16_t* date, uint16_t* time) {  // date/time callback function for SdFat library for timestamping file creation/modification
  DateTime now = RTC.now();
  *date = FAT_DATE(now.year(), now.month(), now.day());
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}
