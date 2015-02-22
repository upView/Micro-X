#include "Wire.h"
#include "I2Cdev.h"
#include "MPU_6050.h"
#include "String.h"
#include "BLDC.h"
#include "PID_.h"

//Sensors
MPU6050 accelgyro; //sensor object
int ax, ay, az; //3-axis accelerometer
int gx, gy, gz; //3-axis gyroscope
int biasX, biasY, biasZ; //gyro offset

//RC Receiver
volatile unsigned long ulStartPeriod; // set in the interrupt
volatile boolean bNewThrottleSignal = false; // set in the interrupt and read in the loop
volatile int rc[7]; //nb of RC channels

//Complementary filter
float pitch, roll;

//Real time constraint
long timer; //general purpose timer 
long timer_old; //timer memory
float G_Dt; //loop time

//Led
int led = 13;

//PID attitude pitch and roll
float command_pitch, command_roll;
float rate_pitch, rate_roll;
PID PID_pitch(&command_pitch, &pitch, &rate_pitch, &G_Dt, 0.5, 0.15, 0.1);
PID PID_roll(&command_roll, &roll, &rate_roll, &G_Dt, 0.5, 0.15, 0.1);

//Throttle
float throttle;

//PID rate yaw
float pid_yaw;
float err_yaw;
float yaw_I;

//Motors
BLDC bldc;

void setup()
{  
  //function calcInput is called everytime there in a change on pin 2. 
  //Pin 2 is hard-wired to interrupt 0
  attachInterrupt(0,calcInput,CHANGE); 
  
  //USB communication enabled. Baud rate 115200 baud 
  Serial.begin(115200);
  
  //I2C enabled
  Wire.begin();
  
  //motors init
  bldc.initializeSpeedController();
  
  //Pin 13 is hard-wired to the red leds. It has to be turned as an output
  pinMode(led, OUTPUT); 
  
  //Sensors init
  accelgyro.initialize();
  
  //gyro calibration to compute offset  
  calib_gyro();
}


void loop()
{
  if((millis()-timer)>=10)   //10ms => 100 Hz loop rate 
  { 
    timer_old = timer; //save time for the next computation
    timer=millis();  //current time
    G_Dt = (timer-timer_old)/1000.0; //Real time of loop run 

    fast_Loop(); //Go to fast loop
  }
}


void fast_Loop(){

   //read sensors and compute angles
  imu_Valget ();


  //compute pid pitch
  command_pitch = -(rc[2]-1120.0)/20;
  rate_pitch = (float)(-gx+biasX)*2000.0f/32768.0f;
  PID_pitch.compute();
  

  //compute pid roll
  command_roll = (rc[1]-1120.0)/20;
  rate_roll = (float)(gy-biasY)*2000.0f/32768.0f;
  PID_roll.compute();


  //compute pid yaw
  err_yaw = (float)(-(gz-biasZ))*2000.0f/32768.0f;
  yaw_I += (float)err_yaw*G_Dt; 
  pid_yaw = err_yaw*1.0+yaw_I;
  
  
  //throttle
  throttle = constrain((rc[0]-1000)/1.8,0,255);
  
  //fail safe
  //when the RC receiver lose Rc transmitter signal, rc[1] is over 1500. 
  if(rc[1]>1500)
  {
    throttle=0;
  }

  //reset values if throttle is low enough for the quadcopter to be on the ground.
  if(throttle < 30)
  {
    //hold pid values to 0 so that the motors are not spining when the throttle stick is down
    PID_pitch.hold();
    PID_roll.hold();
    pid_yaw=0;

    //reset integrators
    PID_pitch.reset_I();
    PID_roll.reset_I();
    yaw_I = 0;
  }

  //debug and tunning
  serial_print();
  
  //output mixer. Each motor speed is a combination between throttle and pid values.
  bldc.setMotorVelocity(REAR_RIGHT, throttle + PID_roll.value() - PID_pitch.value() + pid_yaw);
  bldc.setMotorVelocity(FRONT_LEFT, throttle - PID_roll.value() + PID_pitch.value() + pid_yaw);
  bldc.setMotorVelocity(REAR_LEFT, throttle - PID_roll.value() - PID_pitch.value() - pid_yaw);
  bldc.setMotorVelocity(FRONT_RIGHT, throttle + PID_roll.value() + PID_pitch.value() - pid_yaw);

}


//Interrupt service routine called everytime the digital PPM signal from the RC receiver change on pin 2 
void calcInput()
{
  static unsigned int nThrottleIn;
  static int channel;

  //if the signal is high we start to count
  if(digitalRead(2) == HIGH)
  { 
    ulStartPeriod = micros();
  }
  else
  {
    if(ulStartPeriod)
    {
      nThrottleIn = (int)(micros() - ulStartPeriod); //compute the length of the pulse
      ulStartPeriod = 0; //reset for the next computation

      //channel detector
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



