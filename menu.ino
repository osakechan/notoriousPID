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
    encoderState |= 0b001;
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
  if (programState & 0b000100) {
    lcd.setCursor(0, 2);
    lcd.print(F(" PROFILE IS RUNNING "));
    unsigned long start = millis();
    do { wdt_reset(); mainUpdate(); } while (millis() <= (unsigned long)(start + 1500));  // wait for 1.5 seconds without hard delay
    return;
  }
  char listSize = 2;
  encoderPos = (programState & 0b100000) >> 5;
  char lastReportedPos = encoderPos + 1;
  lcd.setCursor(0, 2);
  lcd.print(F("                    "));
  lcd.setCursor(2, 2);
  lcd.write((byte)1);
  do {
    wdt_reset();
    mainUpdate();
    encoderState |= 0b001;
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

  if (encoderPos) {programState |= 0b100000;}  // set main PID to automatic mode
  else {  // set main PID to manual mode; user entry of main Output for manual only
    programState &= 0b011111;
    do {wdt_reset(); mainUpdate();} while (!digitalRead(pushButton));
    char listSize = 100;
    if (programState & 0b001000) Output = probe::tempCtoF(Output);  // if display unit = deg F, convert Output
    encoderPos = int(Output);
    char lastReportedPos = encoderPos + 1;
    lcd.setCursor(3, 2);
    lcd.print(F("                 "));
    do {  // coarse-grained ajustment (integers)
      wdt_reset();
      mainUpdate();
      encoderState |= 0b001;
      if (lastReportedPos != encoderPos) {
        encoderPos = (encoderPos + listSize) % listSize;
        lcd.setCursor(3, 2);
        lcd.print(encoderPos + Output - int(Output));
        if (programState & 0b001000) lcd.print(F(" \337F"));
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
      encoderState |= 0b001;
      if (lastReportedPos != encoderPos) {
        encoderPos = (encoderPos + listSize) % listSize;
        lcd.setCursor(3, 2);
        lcd.print(Output + double(encoderPos)/10);
        if (programState & 0b001000) lcd.print(F(" \337F"));
          else lcd.print(F(" \337C"));
        lastReportedPos = encoderPos;
      }
      lcd.setCursor(6, 2);
      lcd.cursor();
    } while (digitalRead(pushButton));
    lcd.noCursor();
    Output = Output + double(encoderPos)/10; 
    if (programState & 0b001000) Output = probe::tempFtoC(Output);  // if display is in deg F, convert user entry back to native deg C
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
  if (programState & 0b000100) {
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
  if (programState & 0b001000) Setpoint = probe::tempCtoF(Setpoint);
  encoderPos = int(Setpoint);
  char lastReportedPos = encoderPos + 1;
  do {  // coarse-grained ajustment (integers)
    wdt_reset();
    mainUpdate();
    encoderState |= 0b001;
    if (lastReportedPos != encoderPos) {
      encoderPos = (encoderPos + listSize) % listSize;
      lcd.setCursor(3, 2);
      lcd.print(encoderPos + Setpoint - int(Setpoint));
      if (programState & 0b001000) lcd.print(F(" \337F"));
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
    encoderState |= 0b001;
    if (lastReportedPos != encoderPos) {
      encoderPos = (encoderPos + listSize) % listSize;
      lcd.setCursor(3, 2);
      lcd.print(Setpoint + double(encoderPos)/10);
      if (programState & 0b001000) lcd.print(F(" \337F"));
        else lcd.print(F(" \337C"));
      lastReportedPos = encoderPos;
    }
    lcd.setCursor(6, 2);
    lcd.cursor();
  } while (digitalRead(pushButton));
  lcd.noCursor();
  Setpoint = Setpoint + double(encoderPos)/10;
  if (programState & 0b001000) Setpoint = probe::tempFtoC(Setpoint);

  #if DEBUG == true
    Serial.print(F("main PID Setpoint set to:"));
    Serial.print(Setpoint);
    Serial.print(F(" "));
    Serial.print(freeRAM());
    Serial.println(F(" bytes free SRAM remaining"));
  #endif
}

void heatPIDmode() {
  if (programState & 0b000100) {
    lcd.setCursor(0, 2);
    lcd.print(F(" PROFILE IS RUNNING "));
    unsigned long start = millis();
    do { wdt_reset(); mainUpdate(); } while (millis() <= (unsigned long)(start + 1500));
    return;
  }
  char listSize = 2;
  encoderPos = (programState & 0b010000) >> 4;
  char lastReportedPos = encoderPos + 1;
  lcd.setCursor(0, 2);
  lcd.print(F("                    "));
  lcd.setCursor(2, 2);
  lcd.write((byte)1);
  do {
    wdt_reset();
    mainUpdate();
    encoderState |= 0b001;
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

  if (encoderPos) programState |= 0b010000;  // set heat PID to automatic mode
  else {  // set heat PID to manual mode; user entry of heat Output for manual only
    programState &= 0b101111;
    do {wdt_reset(); mainUpdate();} while (!digitalRead(pushButton));
    char listSize = 100;
    if (programState & 0b001000) heatSetpoint = probe::tempCtoF(heatSetpoint);
    encoderPos = int(heatSetpoint);
    char lastReportedPos = encoderPos + 1;
    lcd.setCursor(3, 2);
    lcd.print(F("                 "));
    do {  // coarse-grained ajustment (integers)
      wdt_reset();
      mainUpdate();
      encoderState |= 0b001;
      if (lastReportedPos != encoderPos) {
        encoderPos = (encoderPos + listSize) % listSize;
        lcd.setCursor(3, 2);
        lcd.print(encoderPos + heatSetpoint - int(heatSetpoint));
        if (programState & 0b001000) lcd.print(F(" \337F"));
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
      encoderState |= 0b001;
      if (lastReportedPos != encoderPos) {
        encoderPos = (encoderPos + listSize) % listSize;
        lcd.setCursor(3, 2);
        lcd.print(heatSetpoint + double(encoderPos)/10);
        if (programState & 0b001000) lcd.print(F(" \337F"));
          else lcd.print(F(" \337C"));
        lastReportedPos = encoderPos;
      }
      lcd.setCursor(6, 2);
      lcd.cursor();
    } while (digitalRead(pushButton));
    lcd.noCursor();
    heatSetpoint = heatSetpoint + double(encoderPos)/10; 
    if (programState & 0b001000) heatSetpoint = probe::tempFtoC(heatSetpoint);
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
  encoderPos = (programState & 0b000010) >> 1;
  char lastReportedPos = encoderPos + 1;
  lcd.setCursor(0, 2);
  lcd.print(F("                    "));
  lcd.setCursor(2, 2);
  lcd.write((byte)1);
  do {
    wdt_reset();
    mainUpdate();
    encoderState |= 0b001;
    if (lastReportedPos != encoderPos) {
      encoderPos = (encoderPos + listSize) % listSize;
      lcd.setCursor(3, 2);
      if (encoderPos) lcd.print(F("Enabled "));
        else lcd.print(F("Disabled"));
      lastReportedPos = encoderPos;
    }
  } while (digitalRead(pushButton));
  if (encoderPos) {
    if (!(programState & 0b000011)) {
      #if DEBUG == true
        Serial.print(F("New logfile pending... "));
        Serial.print(freeRAM());
        Serial.println(F(" bytes free SRAM remaining"));
      #endif

      programState += 3;  // start new LogFile on menu exit
    }
    else if ((programState & 0b000011) == 1) {
      #if DEBUG == true
        Serial.print(F("Pending close operation canceled... "));
        Serial.print(freeRAM());
        Serial.println(F(" bytes free SRAM remaining"));
      #endif
                
      programState = programState & 0b111100 + 2;  // cancel pending file close and leave log running
    }
  }
  else {
    if ((programState & 0b000011) == 2) {
      #if DEBUG == true
        Serial.print(F("Logfile close pending... "));
        Serial.print(freeRAM());
        Serial.println(F(" bytes free SRAM remaining"));
      #endif
              
      programState = (programState & 0b111100) + 1;  // close current LogFile on menu exit
    }
    else if ((programState & 0b000011) == 3) {
      #if DEBUG == true
        Serial.print(F("Pending new operation canceled... "));
        Serial.print(freeRAM());
        Serial.println(F(" bytes free SRAM remaining"));
      #endif

      programState &= 0b111100;  // cancel pending file opening
    }
  }
}

void tempProfile() {              // manage SP profiles
  if (programState & 0b000100) {  // if profile already running
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
      encoderState |= 0b001;
      if (lastReportedPos != encoderPos) {
        encoderPos = (encoderPos + listSize) % listSize;
        lcd.setCursor(16, 2);
        if (encoderPos) lcd.print(F("YES"));
          else lcd.print(F("NO "));
        lastReportedPos = encoderPos;
      }
    } while (digitalRead(pushButton));
    if (encoderPos) {  // empty profile queue, reset program flag and return to main menu
      programState &= 0b111011;
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
    encoderState |= 0b001;
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
    char filename[12] = 
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
    programState |= 0b110100;  //  set PIDs to automatic and enable temperature profile bit
  }
  root.close();
  ProFile.close();
}

void tempUnit() {
  char listSize = 2;
  encoderPos = (programState & 0b001000) >> 3;
  char lastReportedPos = encoderPos + 1;
  lcd.setCursor(0, 2);
  lcd.print(F("                    "));
  lcd.setCursor(2, 2);
  lcd.write((byte)1);
  lcd.print(F(" \337"));
  do {
    wdt_reset();
    mainUpdate();
    encoderState |= 0b001;
    if (lastReportedPos != encoderPos) {
      encoderPos = (encoderPos + listSize) % listSize;
      lcd.setCursor(5, 2);
      if (encoderPos) lcd.print(F("F"));
        else lcd.print(F("C"));
      lastReportedPos = encoderPos;
    }
  } while (digitalRead(pushButton));
  if (encoderPos) programState |= 0b001000;
    else programState &= 0b110111;

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
    encoderState |= 0b001;
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
  if (programState & 0b100000) mainPID.SetMode(AUTOMATIC);
    else mainPID.SetMode(MANUAL);
  if (programState & 0b010000) heatPID.SetMode(AUTOMATIC);
    else heatPID.SetMode(MANUAL);
  if ((programState & 0b000011) == 3) {  // create a new comma seperated value LogFile
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
  if ((programState & 0b000011) == 1) {
    #if DEBUG == true
      Serial.print(F("Logfile closed. "));
      Serial.print(freeRAM());
      Serial.println(F(" bytes free SRAM remaining"));
    #endif

    LogFile.close();  // close LogFile
  }
  programState &= 0b111110;  // reset file change flag
  EEPROMWriteSettings();     // update settings stored in non-volatile memory
}
