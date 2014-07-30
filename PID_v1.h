#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.0.0

class PID {
  public:
    #define AUTOMATIC 1  //Constants used in some of the functions below
    #define MANUAL 0
    #define DIRECT 0
    #define REVERSE 1
    #define RAW 0
    #define FILTERED 1

//commonly used functions **************************************************************************
    PID(double*, double*, double*,         // * constructor.  links the PID to the Input, Output, and 
          double, double, double, int);    // Setpoint.  Initial tuning parameters are also set here

    void SetMode(int Mode);                // * sets PID to either Manual (0) or Auto (non-0)
    bool Compute();                        // * performs the PID calculation.  it should be
                                           // called every time loop() cycles. ON/OFF and
                                           // calculation frequency can be set using SetMode
                                           // SetSampleTime respectively

    void SetOutputLimits(double, double);  //clamps the output to a specific range. 0-255 by default, but
                                           //it's likely the user will want to change this depending on
					   //the application

//available but not commonly used functions ********************************************************
    void SetTunings(double, double, double);  // * While most users will set the tunings once in the 
                             	              // constructor, this function gives the user the option
                                              // of changing tunings during runtime for Adaptive control
    void SetControllerDirection(int);	      // * Sets the Direction, or "Action" of the controller. DIRECT
					      // means the output will increase when error is positive. REVERSE
					      // means the opposite.  it's very unlikely that this will be needed
					      // once it is set in the constructor.
    void SetSampleTime(unsigned long);        // * sets the frequency, in Milliseconds, with which 
                                              // the PID calculation is performed.  default is 100
    void initHistory();                       // init array for calculating slope for D term
    void setOutputType(int);                  // set output type, RAW or FILTERED
    void setFilterConstant(double);           // set filter constant for first order output filter

//Display functions ****************************************************************
    double GetKp();      // These functions query the pid for interal values.
    double GetKi();      // they were created mainly for the pid front-end,
    double GetKd();      // where it's important to know what is actually
    double GetPTerm();   // inside the PID.
    double GetITerm();
    double GetDTerm();
    int GetMode();
    int GetDirection();

  private:
    void Initialize();

    double dispKp;		// * we'll hold on to the tuning parameters in user-entered
    double dispKi;		//   format for display purposes
    double dispKd;
    double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

    int controllerDirection;    // DIRECT = up-up; REVERSE = up-down

    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;             //   This creates a hard link between the variables and the 
    double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.			  
    double PTerm, ITerm, DTerm;          // control output terms
    double lastOutput, FilterConstant;   // for outputing a filtered control signal
    double History[30];                  // for calculating broad PV slope for derivative term
    unsigned long SampleTime, lastTime;  // time between sample/compute (ms), time of last sample (ms)
    double outMin, outMax;     // output constraints
    bool inAuto, isRaw;        // state flags
};
#endif
