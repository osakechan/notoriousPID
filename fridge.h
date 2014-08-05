#ifndef FRIDGE_H
#define FRIDGE_H

#include "Arduino.h"
#include "probe.h"
#include "PID_v1.h"
#include "EEPROMio.h"

enum opState {  // fridge operation states
  IDLE,
  COOL,
  HEAT,
};

const double fridgeIdleDiff = 0.5;       // constrain fridge temperature to +/- 0.5 deg F differential
const double fridgePeakDiff = 1;         // constrain allowed peak error to +/- 1 deg F differential
const unsigned int coolMinOff = 300;     // minimum compressor off time, seconds (5 min)
const unsigned int coolMinOn = 120;      // minimum compressor on time, seconds (2 min)
const unsigned int coolMaxOn = 2700;     // maximum compressor on time, seconds (45 min)
const unsigned int peakMaxTime = 1200;   // maximum runTime to consider for peak estimation, seconds (20 min)
const unsigned int peakMaxWait = 1800;   // maximum wait on peak, seconds (30 min)
const unsigned int heatMinOff = 300;     // minimum HEAT off time, seconds (5 min)
const unsigned int heatWindow = 300000;  // window size for HEAT time proportioning, ms (5 min)

extern byte fridgeState[2];      // [0] - current fridge state; [1] - fridge state t - 1 history
extern double peakEstimator;     // to predict COOL overshoot; units of deg F per hour (always positive)
extern double peakEstimate;      // to determine prediction error = (estimate - actual)
extern unsigned long startTime;  // timing variables for enforcing min/max cycling times
extern unsigned long stopTime;

extern probe fridge, beer;  // external variables define in globals.h
extern double Output, heatSetpoint, heatOutput;
extern const byte relay1, relay2;
extern byte programState;
extern PID heatPID;

void updateFridge();  // core functions
void tuneEstimator(double* estimator, double error);
void updateFridgeState(byte state);
void updateFridgeState(byte state0, byte state1);

inline byte getFridgeState(byte index) { return fridgeState[index]; };  // inlines for accessing fridge variables
inline double getPeakEstimator() { return peakEstimator; };
inline double* getPeakEstimatorAddr() { return &peakEstimator; };
inline unsigned long getStartTime() { return startTime; };
inline unsigned long getStopTime() { return stopTime; };

#endif
