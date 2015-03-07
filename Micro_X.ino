//  _   _ _ __/\   /(_) _____      __
// | | | | '_ \ \ / / |/ _ \ \ /\ / /
// | |_| | |_) \ V /| |  __/\ V  V / 
//  \__,_| .__/ \_/ |_|\___| \_/\_/  
//       |_|                         
// 
// Mini-X Quad control firmware
//
// Copyright (C) 2013-2014 upView
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, in version 3.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.
//
// Micro_X.ino - Containing the main function.


// Platform
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
volatile unsigned long startPeriod; // set in the interrupt
volatile int rc[7]; //nb of RC channels

//Complementary filter
float pitch, roll;

//Real time constraint
long timer; //general purpose timer 
long timer_old; //timer memory
float G_Dt; //loop time

// The red LEDs are conected to Arduino pin n°13
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


// the setup routine runs once
void setup()
{  
  //function calcInput is called everytime there in a falling edge on pin 2. 
  //Pin 2 is hard-wired to interrupt 0
  attachInterrupt(0,calcInput,FALLING); 
  
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


// the loop routine runs forever
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
  command_pitch = -(rc[2]-1484.0)/20;
  rate_pitch = (float)(-gx+biasX)*2000.0f/32768.0f;
  PID_pitch.compute();
  

  //compute pid roll
  command_roll = (rc[1]-1456.0)/20;
  rate_roll = (float)(gy-biasY)*2000.0f/32768.0f;
  PID_roll.compute();


  //compute pid yaw
  err_yaw = (float)(-(gz-biasZ))*2000.0f/32768.0f;
  yaw_I += (float)err_yaw*G_Dt; 
  pid_yaw = err_yaw*1.0+yaw_I;
  
  
  //throttle
  throttle = constrain((rc[0]-1180)/1.8,0,255);
  
  //fail safe
  //when the RC receiver lose Rc transmitter signal, rc[1] is over 1500. 
  if(rc[1]>1900)
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


//Interrupt service routine called everytime the digital PPM signal from the RC receiver is falling on pin 2 
void calcInput()
{
  //static variables are not reset when we exit the function
  static unsigned int pulseIn;
  static int channel;
  
      //length of current pulse
      pulseIn = (int)(micros() - startPeriod);
      
      //remember the time for next loop
      startPeriod = micros();

      //channel detector
      if(pulseIn >2000){
        channel = 0;
      }
      //store value
      else
      {
        rc[channel]=pulseIn;
        channel++; //increment channel for next time
      }
}



