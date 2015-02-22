#include "BLDC.h"

/** Assign PWM pin-out to each ESC controlling the motors.
 */
BLDC::BLDC()
{
    _motorsPins[FRONT_LEFT]  = BLDC_PWM_MOTOR_FRONT_LEFT;
    _motorsPins[FRONT_RIGHT] = BLDC_PWM_MOTOR_FRONT_RIGHT;
    _motorsPins[REAR_LEFT]   = BLDC_PWM_MOTOR_REAR_LEFT;
    _motorsPins[REAR_RIGHT]  = BLDC_PWM_MOTOR_REAR_RIGHT;


}

/** TODO: add description here.
 */
void BLDC::initializeSpeedController()
{
        for (unsigned int i = 0; i < BLDC_MOTORS_COUNT; i++) 
    {
        pinMode(_motorsPins[i], OUTPUT);
    }
}

/** TODO: add description here.
 */
void BLDC::calibrateSpeedController()
{
    //TODO: implement here for ESC
}

/** Set specified velocity to specified motor.
 * @param idMotor Motor number see MOTORS_ASSIGNEMENT
 * @param nbPulse PWM applied to Speed Controller (PWM)
 */
void BLDC::setMotorVelocity(uint8_t idMotor, uint8_t nbPulse)
{
    analogWrite(_motorsPins[idMotor], nbPulse);
}

/** Set specified velocity to all motors.
 * @param devAddr I2C slave device address
 */
void BLDC::setAllMotorsVelocity(uint8_t nbPulse)
{
    for (unsigned int i = 0; i < BLDC_MOTORS_COUNT; i++) 
    {
        setMotorVelocity(i, constrain(nbPulse, BLDC_MOTORS_MINIMUM_SPEED, BLDC_MOTORS_MAXIMUM_SPEED));
    }
}

/** Read a single bit from an 8-bit device register.
 */
void BLDC::killAllMotors()
{
    setAllMotorsVelocity(BLDC_MOTORS_MINIMUM_SPEED);
}
