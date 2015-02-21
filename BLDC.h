#ifndef _HAL_BLDC_
#define _HAL_BLDC_

#include "easyI2C.h"

// ESC PWM pinout
#define BLDC_PWM_MOTOR_FRONT_LEFT   3
#define BLDC_PWM_MOTOR_FRONT_RIGHT  9
#define BLDC_PWM_MOTOR_REAR_LEFT    10
#define BLDC_PWM_MOTOR_REAR_RIGHT   11

#define BLDC_MOTORS_COUNT 4

enum { FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT };

class BLDC
{
    public:
        BLDC();
        void initializeSpeedController();
        void calibrateSpeedController();
        void setMotorVelocity();
        void killAllMotors();

	private:
        uint8_t _motorsPins[BLDC_MOTORS_COUNT];
};

#endif


