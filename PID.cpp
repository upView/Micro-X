

#include "PID_.h"

/** Constructor.
 * @param command New angle demand
 * @param angle New current angle
 * @param rate New angular speed
 * @param time New loop time
 * @param P New P gain
 * @param I New I gain
 * @param D New D gain
 */
PID::PID(float* command, float* angle, float* rate, float* time, float P, float I, float D)
{	
    _command = command;
    _angle = angle;
    _time = time;
    _rate = rate;
    _P = P;
    _I = I;
    _D = D;				
}
 
/** Update PID value.
 */
void PID::compute()
{
      float error = *_command - *_angle;
      
      _ITerm+= error * *_time;

      _pid = _P * error + _I * _ITerm + _D * *_rate;
}

/** Reset PID Integral term.
 */
void PID::reset_I()
{
	_ITerm = 0;
}

/** Get PID value.
 * @return _pid.
 */
float PID::value()
{
  return _pid;
}

/** Reset PID value.
 */
void PID::hold()
{
  _pid = 0;
}

/** Get P gain value.
 * @return _P.
 */
float PID::getP()
{
  return _P;
}

/** Get I gain value.
 * @return _I.
 */
float PID::getI()
{
  return _I;
}

/** Get D gain value.
 * @return _D.
 */
float PID::getD()
{
  return _D;
}

/** Set P gain value.
 * @param P New P gain.
 */
void PID::setP(float P)
{
 _P = P;
}

/** Set I gain value.
 * @param I New I gain.
 */
void PID::setI(float I)
{
 _I = I;
}

/** Set D gain value.
 * @param D New D gain.
 */
void PID::setD(float D)
{
  _D = D;
}
