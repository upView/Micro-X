#ifndef _HAL_BLDC_
#define _HAL_BLDC_

#include "Arduino.h"

#define BLDC_MOTORS_COUNT           4
#define BLDC_MOTORS_MINIMUM_SPEED   0
#define BLDC_MOTORS_MAXIMUM_SPEED   255

#define BLDC_PWM_MOTOR_FRONT_LEFT   9
#define BLDC_PWM_MOTOR_FRONT_RIGHT  11
#define BLDC_PWM_MOTOR_REAR_LEFT    10
#define BLDC_PWM_MOTOR_REAR_RIGHT   3

enum MOTORS_ASSIGNEMENT
{
    FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT
};

class BLDC
{
    public:
        BLDC();
        void initializeSpeedController();
        void calibrateSpeedController();

        void setMotorVelocity(uint8_t idMotor, uint8_t nbPulse);
        void setAllMotorsVelocity(uint8_t nbPulse);
        void killAllMotors();

	private:
        uint8_t _motorsPins[BLDC_MOTORS_COUNT];
};

#endif


