#ifndef _HAL_MPU6050_
#define _HAL_MPU6050_

#include "easyI2C.h"

// Device I2C adress
#define MPU6050_ADDRESS             0x68

// Measurements registers
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48

// Power management
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_PWR1_SLEEP_BIT         6
#define MPU6050_PWR1_CLKSEL_BIT        2
#define MPU6050_PWR1_CLKSEL_LENGTH     3

// Gyroscope sensibility
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_GCONFIG_FS_SEL_BIT     4
#define MPU6050_GCONFIG_FS_SEL_LENGTH  2
#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

// Accelerometer sensibility
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_ACONFIG_AFS_SEL_BIT    4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH 2
#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

// Internal clock
#define MPU6050_CLOCK_INTERNAL      0x00
#define MPU6050_CLOCK_PLL_XGYRO     0x01
#define MPU6050_CLOCK_PLL_YGYRO     0x02
#define MPU6050_CLOCK_PLL_ZGYRO     0x03
#define MPU6050_CLOCK_PLL_EXT32K    0x04
#define MPU6050_CLOCK_PLL_EXT19M    0x05
#define MPU6050_CLOCK_KEEP_RESET    0x07

// Internal LPF (Low-pass filter) setting
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_CFG_DLPF_CFG_BIT       2
#define MPU6050_CFG_DLPF_CFG_LENGTH    3
#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

class MPU6050
{
    public:
        MPU6050();
        void initialize();
        void setDLPFMode(uint8_t bandwidth);
        void setFullScaleGyroRange(uint8_t range);
        void setFullScaleAccelRange(uint8_t range);
		void setSleepEnabled(bool enabled);
		void setClockSource(uint8_t source);
		void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
        
	private:
        uint8_t devAddr;
        uint8_t buffer[14];
};

#endif


