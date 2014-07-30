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
  if (programState & 0b001000) lcd.print(F("\337F "));
    else lcd.print(F("\337C "));
  if (programState & 0b000100) lcd.print(F("PGM "));
  else {
    if (programState & 0b100000) lcd.print(F("A "));
      else lcd.print(F("M "));
    if (programState & 0b010000) lcd.print(F("A "));
      else lcd.print(F("M "));
  }
  if (fridgeState[0] == IDLE) lcd.print(F("I "));
  if (fridgeState[0] == HEAT) lcd.print(F("H "));
  if (fridgeState[0] == COOL) lcd.print(F("C "));
  if (programState & 0b000010) lcd.print(F("SD"));
    else { lcd.write((byte)5); lcd.write((byte)5); }
  if (!encoderPos) {
    DateTime time = RTC.now();
    lcd.setCursor(11, 2);
    /*lcd.print((time.month() - (time.month() % 10))/10);
    lcd.print(time.month() % 10);
    lcd.print(F("."));
    lcd.print((time.day() - (time.day() % 10))/10);
    lcd.print(time.day() % 10);
    lcd.print(F("  "));*/
    lcd.print((time.hour() - (time.hour() % 10))/10);
    lcd.print(time.hour() % 10);
    lcd.print(F(":"));
    lcd.print(time.minute()/10 % 6);
    lcd.print(time.minute() % 10);
    lcd.print(F(":"));
    lcd.print(time.second()/10 % 6);
    lcd.print(time.second() % 10);
  }
  if (programState & 0b001000) {  // temperature units = deg F
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
        switch (fridgeState[0]) {
          case IDLE:
            if (fridgeState[1] == COOL) lcd.print(F("    wait on peak    "));
              else lcd.print(F("       idling       "));
            elapsed = (double)(millis() - stopTime) / 60000;   // time since IDLE start in min
            break;

          case COOL:
            lcd.print(F("       cooling      "));
            elapsed = (double)(millis() - startTime) / 60000;  // time since COOL start in min
            break;

          case HEAT:
            elapsed = millis() - startTime;  // time since HEAT window start in ms
            if (elapsed < heatOutput) lcd.print(F("      heating      "));
              else lcd.print(F("    idle on heat    "));
            elapsed /= 60000;  // convert ms to min
            break;
        }
        lcd.setCursor(4, 2);
        lcd.print(peakEstimator);
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
        switch (fridgeState[0]) {
          case IDLE:
            if (fridgeState[1] == COOL) lcd.print(F("    wait on peak    "));
              else lcd.print(F("       idling       "));
            elapsed = (double)(millis() - stopTime) / 60000;   // time since IDLE start in min
            break;

          case COOL:
            lcd.print(F("       cooling      "));
            elapsed = (double)(millis() - startTime) / 60000;  // time since COOL start in min
            break;

          case HEAT:
            elapsed = millis() - startTime;  // time since HEAT window start in ms
            if (elapsed < heatOutput) lcd.print(F("      heating      "));
              else lcd.print(F("    idle on heat    "));
            elapsed /= 60000;  // convert ms to min
            break;
        }
        lcd.setCursor(4, 2);
        lcd.print(peakEstimator);
        lcd.setCursor(14, 2);
        lcd.print(elapsed);
        break;
    }
  }
}
