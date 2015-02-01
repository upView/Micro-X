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
// main.ino - Containing the main function.

// Platform
#include "Wire.h"
#include "easyI2C.h"

// HAL
#include "MPU_6050.h"
// +
// 

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

volatile unsigned long ulStartPeriod = 0; // set in the interrupt
volatile boolean bNewThrottleSignal = false; // set in the interrupt and read in the loop
volatile int rc[7];

float pitch, roll;
float G_Dt=0.02;

float accel_lin_x, accel_lin_y, accel_lin_z;
float vit_rot_x, vit_rot_y, vit_rot_z;

int led = 13;

long timer=0; //general purpose timer 
long timer_old;

float offset_x, offset_y, offset_z;

float command_pitch, err_pitch, pid_pitch, pitch_I, pitch_D;

float command_roll, err_roll, pid_roll, roll_I, roll_D;

float throttle;

float pid_yaw;

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
  
  pinMode(led, OUTPUT); 
  
  accelgyro.setSleepEnabled(false);
  accelgyro.setFullScaleGyroRange(3); //Gyro scale 2000deg/s
  accelgyro.setFullScaleAccelRange(1);//Accel scale 4g
  accelgyro.setClockSource(3);// Select GyroZ clock
  accelgyro.setDLPFMode(4);// set bandwidth of both gyro and accelerometer to ~20 Hz

  calibration_gyroscope();

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


void fast_Loop() 
{

  //Lecture des valeurs brutes des données des capteurs.
  //Conversion en unité SI
  lecture_capteur_et_conversion();
  
  //Calcul des angle de pitch et roll
  filtre_complementaire();


  //pid pitch = rc1
  command_pitch = -(rc[2]-1120.0)/20;
  err_pitch = command_pitch - pitch;
  pitch_D = -vit_rot_x;
  pitch_I += (float)err_pitch*G_Dt; 
  pitch_I = constrain(pitch_I,-50,50);
  pid_pitch = err_pitch*kp+pitch_I*ki+pitch_D*kd;


  //ROLL
  command_roll = (rc[1]-1120.0)/20;
  err_roll = command_roll - roll;
  roll_D = vit_rot_y;
  roll_I += (float)err_roll*G_Dt; 
  roll_I = constrain(roll_I,-50,50);
  pid_roll = err_roll*kp+roll_I*ki+roll_D*kd;//pid_roll=0; //à supprimer


  //YAW
  pid_yaw = -vit_rot_z;
  
  
  //THROTTLE
  throttle = constrain((rc[0]-1000)/1.8,0,255);
  
  
  //Sécurité en cas de perte du signal de la manette. 
  //Quand le recepteur RC perd le contacte avec la manette, la voie rc[1] prends une valeurs > 1500, 
  //valeur qui ne peut jamais etre atteinte en temps normal. On éteind immediatement les moteurs.
  if(rc[1]>1500)
  {
    throttle=0;
  }

  //tant que le throttle est < 30 on est au dessous du point de décollage
  if(throttle < 30)
  {
    //les pid n'ont pas d'influence
    pid_pitch=0;
    pid_roll=0;
    pid_yaw=0;

    //réinitialisation des intégrateurs
    pitch_I=0;
    roll_I=0;
  }

  //Envoie ou reception d'information par communication série 
  communication_serie();
  
  //Commande des moteurs. Mis à jour des periodes des signaux pwm.
  analogWrite(3, constrain(throttle+pid_roll-pid_pitch+pid_yaw,0,255)); //arrière droit
  analogWrite(9, constrain(throttle-pid_roll+pid_pitch+pid_yaw,0,255)); //avant gauche
  analogWrite(10, constrain(throttle-pid_roll-pid_pitch-pid_yaw,0,255)); //arrière gauche
  analogWrite(11, constrain(throttle+pid_roll+pid_pitch-pid_yaw,0,255)); //avant droit
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