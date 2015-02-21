

#include "PID_.h"

PID::PID(float* command, float* angle, float* pid, float* rate, float* time, float P, float I, float D)
{	
    _command = command;
    _angle = angle;
    _pid = pid;
    _time = time;
    _rate = rate;
    _P = P;
    _I = I;
    _D = D;				
}
 

void PID::Compute()
{
      float error = *_command - *_angle;
      
      ITerm+= error * *_time;

      *_pid = _P * error + _I* ITerm + _D * *_rate;
}

void PID::Reset_I()
{
	ITerm = 0;
}
