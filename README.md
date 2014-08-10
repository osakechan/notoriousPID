notorious-PID
=============

PID fermentation control for AVR platforms

notorious PID is an open source fermentation temperature control program for homebrew use.  The code has been developed using the arduino IDE.

###List of Physical Components:

- [Arduino MEGA 2560](http://arduino.cc/en/Main/arduinoBoardMega2560)
- [Adafruit data logging shield](http://www.adafruit.com/product/1141)
- [20x4 character LCD](http://www.adafruit.com/product/198)
- [Rotary encoder with pushbutton](http://www.adafruit.com/product/377)
- [SainSmart 2 channel relay board](http://www.sainsmart.com/arduino-pro-mini.html)
- 2x [DS18B20 Dallas OneWire digital temperature sensor](http://www.adafruit.com/product/381)
- [3/8" stainless thermowell](https://www.brewershardware.com/12-Stainless-Steel-Thermowell-TWS12.html)
- [A refrigerator or chest freezer](http://www.craigslist.org/about/sites)
- [Flexwatt heat tape](http://www.calorique.com/en/flexwatt-heat-tape/)

###Logic Circuitry:
![nPID logic wiring](https://raw.githubusercontent.com/osakechan/notorious-PID/master/img/nPID%20wiring%20layout.png)

*a couple notes about the above diagram:*
- the prototyping shield has been substituted for the adafruit data logging shield (same footprint)
- this project uses a 20x4 character lcd with backlight (shown is the stock 16x2)
- the lcd utilizes a trim-pot to adjust contrast (dashed blue wire)
- the rotary encoder from adafruit incorporates a pushbutton (shown seperate)
- the onewire data line requires a 4.7kohm pullup (dashed green wire)
- external power *must* be supplied to the onewire sensors (cannot use parasite power)
- there is no template for the sainsmart relay board but wiring should be straight-forward

###Connection List:
| Hardware | AVR Pin |.| Hardware | AVR Pin |
|--------|---|-|--------|---|
| Rotary A Channel | 3 |.| LCD D7 | 4 |
| Rotary B Channel | 4 |.| LCD D6 | 5 |
| Rotary Pushbutton | A0 |.| LCD D5 | 6 |
| OneWire Data | A1 |.| LCD D4 | 7 |
| Relay 1 (fridge) | A2 |.|.|.|
| Relay 2 (compressor) | A3 |.|.|.|

###Power Circuitry:
![nPID power wiring](https://raw.githubusercontent.com/osakechan/notoriousPID/master/img/nPid%20power%20layout.png)

*a couple notes about the above diagram:*
- *mains* outlet provides project-wide GFCI protection
- plug refrigerator into *cooling* outlet (duh)
- plug heat element into *heating* outlet (duh)
- plug arduino (and optional fans, etc.) into *mains* outlet

###Control Overview:

  A standard PID control algorithm outputs the air temperature necessary to maintain a desired fermentation setpoint. Controller output of the main PID cascades into two additional control algorithms for heating and cooling.  Final control elements are the refrigerator compressor and resistive heating element.  Temperature sensing of fermenting beer and chamber air is performed by the Dallas OneWire DS18B20.  The sensor's on-board DAC performs a conversion with up to 12-bit resolution.  Potentially limitless numbers of Dallas OneWire Sensors may be attached via a single digital input in on the AVR.

  **Cooling** --  The refrigerator compressor is switched by a differential control algorithm with time-based overshoot prediction capabilities.

  **Heating** --  A second PID instance outputs a duty cycle for time proportioned control of a resistive heating element lining the inner chamber walls.

###Additonal Features:

  **EEPROM storage** -- notorious PID stores vital program states and settings in non-volatile EEPROM memory space.  If power is lost or the arduino reboots via the reset button, previous settings can be recalled from EEPROM at startup.

  **Data Logging** -- Logging functionality is provided by the Adafruit data logging shield.  The shield includes an SD card slot and a real time clock for accurate timestamping of data and files.  Logfiles are formatted as simple CSV with headers.  Logging operations may be enabled/disabled by the end user at any time via the menu.
  
  **Temperature Profiles** -- The program includes support for end-user created temperature profiles.  Profiles in CSV format may be placed in the /PROFILES/ directory of the SD card used for data logging.  Files use the 8.3 filename format with .PGM file extension and consist of comma separated pairs of setpoint temperature (deg C) and duration (hours).  During profile operation, main PID setpoint is varied according to the pairs included in the .PGM file.  Profiles may be enabled/disabled via the menu.
  
  **Watchdog Failsafe** -- An infinite loop or other AVR lock-up could lead to a loss of control of the final control elements.  To prevent an AVR failure from leading to unsafe operation, notorious PID makes use of the Watchdog timer feature of arduino (and similar) boards.  The Watchdog is an onboard countdown timer that will reboot the arduino if it has not recieved a reset pulse from the AVR within a set time.
  
###Future Features:

  **WiFi Connectivity** -- Connectivity to be acomplished via the Adafruit wifi breakout with external antenna.  Data will be viewable online via the Xively service.

-----------------------

Many thanks go to Elco Jacobs who, without even knowing it, greatly influenced the direction of this project with his work on both [UberFridge](http://www.elcojacobs.com/uberfridge/) and [brewPI](http://www.brewpi.com/).  Also, thanks to the many helpful users on both the Arduino and Adafruit forums!
