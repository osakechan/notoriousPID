void updateFridge() {                  // maintain fridge at temperature set by mainPID -- COOLing with predictive differential, HEATing with time proportioned heatPID
  switch (fridgeState[0]) {  // MAIN switch -- IDLE/peak detection, COOL, HEAT routines
    default:
    case IDLE:
      if (fridgeState[1] == IDLE) {   // only switch to HEAT/COOL if not waiting for COOL peak
        if ((fridge.getFilter() > Output + fridgeIdleDiff) && ((unsigned long)((millis() - stopTime) / 1000) > coolMinOff)) {  // switch to COOL only if temp exceeds IDLE range and min off time met
          updateFridgeState(COOL);    // update current fridge status and t - 1 history
          digitalWrite(relay1, LOW);  // close relay 1; supply power to fridge compressor
          startTime = millis();       // record COOLing start time
        }
        else if ((fridge.getFilter() < Output - fridgeIdleDiff) && ((unsigned long)((millis() - stopTime) / 1000) > heatMinOff)) {  // switch to HEAT only if temp below IDLE range and min off time met
          updateFridgeState(HEAT);
          if (programState & 0b010000) heatSetpoint = Output;  // update heat PID setpoint if in automatic mode
          heatPID.Compute();      // compute new heat PID output, update timings to align PID and time proportioning routine
          startTime = millis();   // start new time proportioned window
        }
      }
      else if (fridgeState[1] == COOL) {  // do peak detect if waiting on COOL
        if (fridge.peakDetect()) {               // negative peak detected...
          tuneEstimator(&peakEstimator, peakEstimate - fridge.getFilter());  // (error = estimate - actual) positive error requires larger estimator; negative:smaller
          fridgeState[1] = IDLE;          // stop peak detection until next COOL cycle completes
        }
        else {                                                               // no peak detected
          double offTime = (unsigned long)(millis() - stopTime) / 1000;      // IDLE time in seconds
          if (offTime < peakMaxWait) break;                                  // keep waiting for filter confirmed peak if too soon
          tuneEstimator(&peakEstimator, peakEstimate - fridge.getFilter());  // temp is drifting in the right direction, but too slowly; update estimator
          fridgeState[1] = IDLE;                                             // stop peak detection
        }
      }
      break;

    case COOL:  // run compressor until peak predictor lands on controller Output
      { double runTime = (unsigned long)(millis() - startTime) / 1000;  // runtime in seconds
      if (runTime < coolMinOn) break;     // ensure minimum compressor runtime
      if (fridge.getFilter() < Output) {  // temp already below output: most likely cause is change in setpoint or long minimum runtime
        updateFridgeState(IDLE, IDLE);    // go IDLE, ignore peaks
        digitalWrite(relay1, HIGH);       // open relay 1; power down fridge compressor
        stopTime = millis();              // record idle start
        break;
      }
      if ((fridge.getFilter() - (min(runTime, peakMaxTime) / 3600) * peakEstimator) <= Output) {  // if estimated peak lands on Output, set IDLE and wait for actual peak
        peakEstimate = fridge.getFilter() - (runTime / 3600) * peakEstimator;   // record estimated peak prediction
        updateFridgeState(IDLE);     // go IDLE, wait for peak
        digitalWrite(relay1, HIGH);
        stopTime = millis();
      }
      if (runTime > coolMaxOn) {  // if compressor runTime exceeds max on time, skip peak detect, go IDLE
        updateFridgeState(IDLE, IDLE);
        digitalWrite(relay1, HIGH);
        stopTime = millis();
      }
      break; }

    case HEAT:  // run HEAT using time proportioning
      { double runTime = millis() - startTime;  // runtime in ms
      if ((runTime < heatOutput) && digitalRead(relay2)) digitalWrite(relay2, LOW);           // active duty; close relay, write only once
        else if ((runTime > heatOutput) && !digitalRead(relay2)) digitalWrite(relay2, HIGH);  // active duty completed; rest of window idle; write only once
      if (programState & 0b010000) heatSetpoint = Output;
      if (heatPID.Compute()) {  // if heatPID computes (once per window), current window complete, start new
        startTime = millis();
      }
      if (fridge.getFilter() > Output + fridgeIdleDiff) {  // temp exceeds setpoint, go to idle to decide if it is time to COOL
        updateFridgeState(IDLE, IDLE);
        digitalWrite(relay2, HIGH);
        stopTime = millis();
      }
      break; }
  }
}

void tuneEstimator(double* estimator, double error) {  // tune fridge overshoot estimator
  if (abs(error) <= fridgePeakDiff) return;            // leave estimator unchanged if error falls within peak differential (+/-1 deg)
  if (error > 0) *estimator *= constrain(1.2 + 0.03 * abs(error), 1.2, 1.5);                 // if positive error; increase estimator 20% - 50% relative to error
    else *estimator = max(0.05, *estimator / constrain(1.2 + 0.03 * abs(error), 1.2, 1.5));  // if negative error; decrease estimator 17% - 33% relative to error, constrain to non-zero value
  EEPROMWrite(38, peakEstimator, DOUBLE);              // update estimator value stored in EEPROM
}

void updateFridgeState(byte state) {  // update current fridge state
  fridgeState[1] = fridgeState[0];
  fridgeState[0] = state;
}

void updateFridgeState(byte state0, byte state1) {  // update current fridge state and history
  fridgeState[1] = state1;
  fridgeState[0] = state0;
}
