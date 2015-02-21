
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU_6050.h"
#include "String.h"
#include "BLDC.h"

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

MPU6050 accelgyro;
BLDC bldc;

int16_t ax, ay, az;
int16_t gx, gy, gz;

volatile unsigned long ulStartPeriod = 0; // set in the interrupt
volatile boolean bNewThrottleSignal = false; // set in the interrupt and read in the loop
volatile int rc[7];

float pitch, roll;
float G_Dt=0.02;

int led = 13;

long timer=0; //general purpose timer 
long timer_old;

float biasX, biasY, biasZ;

float command_pitch;
float err_pitch;
float pid_pitch;
float pitch_I;
float pitch_D;
float pitch_D_com;
float err_pitch_old;

float command_roll;
float err_roll;
float pid_roll;
float roll_I;
float roll_D;
float roll_D_com;
float err_roll_old;

float throttle;

float pid_yaw;
float err_yaw;
float yaw_I;

float kp = 0.5;
float ki = 0.15; 
float kd = 0.1;

void setup()
{  
  pinMode(3, OUTPUT);  
  pinMode(9, OUTPUT);  
  pinMode(10, OUTPUT);  
  pinMode(11, OUTPUT); 


  attachInterrupt(0,calcInput,CHANGE);
  Serial.begin(115200);
  Wire.begin();
  
  bldc.initializeSpeedController();
  
  pinMode(led, OUTPUT); 
  accelgyro.setSleepEnabled(false);
  
 accelgyro.setFullScaleGyroRange(3); //Gyro scale 2000deg/s
  delay(1);
  accelgyro.setFullScaleAccelRange(1);//Accel scale 4g
  delay(1);
  accelgyro.setClockSource(3);// Select GyroZ clock
  delay(1);
  accelgyro.setDLPFMode(4);// set bandwidth of both gyro and accelerometer to ~20 Hz
  delay(1);
  

  calib_gyro();

  timer = millis();
  delay(20);
}


void loop()
{
  if((millis()-timer)>=10)   // 10ms => 100 Hz loop rate 
  { 
    timer_old = timer;
    timer=millis();
    G_Dt = (timer-timer_old)/1000.0;      // Real time of loop run 

    fast_Loop();

  }
}


void fast_Loop(){

  imu_Valget (); // read sensors

  


  //pid pitch = rc1
  command_pitch = -(rc[2]-1120.0)/20;
  err_pitch = command_pitch - pitch;
  pitch_D = (float)(-gx+biasX)*2000.0f/32768.0f;
  pitch_I += (float)err_pitch*G_Dt; 
  pitch_I = constrain(pitch_I,-50,50);
  pid_pitch = err_pitch*kp+pitch_I*ki+pitch_D*kd; //P=10 I=15 D=5 was good //D=8 the limit //P=15 I=30 D=5
  //pid_pitch = 0;

  //ROLL
  command_roll = (rc[1]-1120.0)/20;
  err_roll = command_roll - roll;
  roll_D = (float)(gy-biasY)*2000.0f/32768.0f;
  roll_I += (float)err_roll*G_Dt; 
  roll_I = constrain(roll_I,-50,50);
  pid_roll = err_roll*kp+roll_I*ki+roll_D*kd; //P=0.1 I=0.85 D=0.1
  //pid_roll=0; //à supprimer

  //YAW
  err_yaw = (float)(-(gz-biasZ))*2000.0f/32768.0f;
  yaw_I += (float)err_yaw*G_Dt; 
  pid_yaw = err_yaw*1.0+yaw_I;
  //pid_yaw=0;
  
  //Throttle
  throttle = constrain((rc[0]-1000)/1.8,0,255);
  //fail safe
  if(rc[1]>1500)
  {
    throttle=0;
  }

  if(throttle < 30)
  {
    pid_pitch=0;
    pid_roll=0;
    pid_yaw=0;

    pitch_I=0;
    roll_I=0;
  }

  IMU_print();
  
  bldc.setMotorVelocity(REAR_RIGHT, throttle + pid_roll - pid_pitch + pid_yaw);
  bldc.setMotorVelocity(FRONT_LEFT, throttle - pid_roll + pid_pitch + pid_yaw);
  bldc.setMotorVelocity(REAR_LEFT, throttle - pid_roll - pid_pitch - pid_yaw);
  bldc.setMotorVelocity(FRONT_RIGHT, throttle + pid_roll + pid_pitch - pid_yaw);
/*
  analogWrite(3, constrain(throttle+pid_roll-pid_pitch+pid_yaw,0,255));//arrière droit
  analogWrite(9, constrain(throttle-pid_roll+pid_pitch+pid_yaw,0,255));//avant gauche
  analogWrite(10, constrain(throttle-pid_roll-pid_pitch-pid_yaw,0,255));//arrière gauche
  analogWrite(11, constrain(throttle+pid_roll+pid_pitch-pid_yaw,0,255));//avant droit
*/

/*
   analogWrite(3, 0);//arrière droit
   analogWrite(9, 0);//avant gauche
   analogWrite(10, 0);//arrière gauche
   analogWrite(11, 0);//avant droit
*/
}

void calcInput()
{
  static unsigned int nThrottleIn;
  static int channel;

  if(digitalRead(2) == HIGH)
  { 
    ulStartPeriod = micros();
  }
  else
  {
    if(ulStartPeriod)
    {
      nThrottleIn = (int)(micros() - ulStartPeriod);
      ulStartPeriod = 0;

      if(nThrottleIn >2000){
        channel = 0;
      }
      else
      {
        rc[channel]=nThrottleIn;
        channel++;
      }
    }
  }
}



