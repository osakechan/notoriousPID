#ifndef PROBE_H
#define PROBE_H

#include "Arduino.h"
#include <Serial.h>
#include <avr/wdt.h>
#include <OneWire.h>

class probe {
    static OneWire* _myWire;
    static double _sampleHz;
    static boolean _sampled;
    static unsigned long _lastSample;
    static unsigned int _offset;

    byte _address[8];
    double _temperature[4];
    double _filter[4];

    static boolean _isConv() { return !_myWire->read(); }
    boolean _getAddr();
    boolean _updateTemp();
    void _updateFilter();
    
  public:
    probe(OneWire* onewire) { if (!_myWire) _myWire = onewire; _getAddr(); }
    void init();
    void update();
    boolean peakDetect();
    double getTemp() { return _temperature[0]; }
    double getFilter() { return _filter[0]; }

    static void setSampleHz(double hz) { _sampleHz = hz; }
    static boolean isReady();
    static void startConv();
    static double tempCtoF(double tempC) { return ((tempC * 9 / 5) + 32); }
    static double tempFtoC(double tempF) { return ((tempF - 32) * 5 / 9); }
};

#endif
