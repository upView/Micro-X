

class PID
{
public:

  //commonly used functions **************************************************************************
  PID(float* command, float* angle, float* pid, float* rate, float* time, float P, float I, float D);     //   Setpoint.  Initial tuning parameters are also set here

  void Compute();

  float GetKp();						  // These functions query the pid for interal values.
  float GetKi();						  //  they were created mainly for the pid front-end,
  float GetKd();						  // where it's important to know what is actually 				  //
  void Reset_I();


  float _P;                  // * (P)roportional Tuning Parameter
  float _I;                  // * (I)ntegral Tuning Parameter
  float _D;                  // * (D)erivative Tuning Parameter

  float *_command;              // * Pointers to the Input, Output, and Setpoint variables
  float *_angle;             //   This creates a hard link between the variables and the 
  float *_pid;
  float *_time;
  float *_rate;
  
  float ITerm;

};


