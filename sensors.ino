

void imu_Valget ()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float pitchAcc, rollAcc;

  // Integrate the gyroscope data -> int(angularSpeed) = angle
  pitch += ((float)(gx-biasX)*2000.0f/32768.0f) * G_Dt; // Angle around the X-axis
  roll -= ((float)(gy-biasY)*2000.0f/32768.0f) * G_Dt; // Angle around the Y-axis

  // Compensate for drift with accelerometer data if no vibration
  // Weight for accelerometer info (<0.75G = 0.0, 1G = 1.0 , >1.25G = 0.0)
  int forceMagnitudeApprox = abs(ax) + abs(ay) + abs(az);

  if (forceMagnitudeApprox > 6144 && forceMagnitudeApprox < 10240)
  {
    // Turning around the X axis results in a vector on the Y-axis
    pitchAcc = atan2((float)ay, (float)az) * 180 / M_PI;
    pitch = pitch * 0.98 + pitchAcc * 0.02;

    // Turning around the Y axis results in a vector on the X-axis
    rollAcc = atan2((float)ax, (float)az) * 180 / M_PI;
    roll = roll * 0.98 + rollAcc * 0.02;
  }
 
}



void calib_gyro()
{
  for(int c=0; c<10; c++)
  { 
    digitalWrite(led, HIGH);
    delay(200);
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    digitalWrite(led, LOW); 
    delay(200);
  }

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  biasX = gx;
  biasY = gy;
  biasZ = gz;

  for(int i=0;i<100;i++)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    biasX = biasX*0.80 + gx*0.20;
    biasY = biasY*0.80 + gy*0.20;
    biasZ = biasZ*0.80 + gz*0.20;
    delay(20);
  }
}


void serial_print ()
{

  Serial.print(PID_pitch.getP());
  Serial.print(" ");
  Serial.print(PID_pitch.getI());
Serial.print(" ");
  Serial.print(PID_pitch.getD());
  Serial.println(" ");
  
  
  while(Serial.available())
  {
    char c = Serial.read();
    
    if (c=='a')
   {
     PID_pitch.setP(PID_pitch.getP()+0.02);
     PID_roll.setP(PID_roll.getP()+0.02);
   } 
   else if (c=='z')
   {
    PID_pitch.setP(PID_pitch.getP()-0.02);
    PID_roll.setP(PID_roll.getP()-0.02);
   }
    else if (c=='q')
   {
     PID_pitch.setI(PID_pitch.getI()+0.02);
     PID_roll.setI(PID_roll.getI()+0.02);
   }
    else if (c=='s')
   {
    PID_pitch.setI(PID_pitch.getI()-0.02);
    PID_roll.setI(PID_roll.getI()-0.02);
   }
       else if (c=='w')
   {
     PID_pitch.setD(PID_pitch.getD()+0.02);
     PID_roll.setD(PID_roll.getD()+0.02);
   }
    else if (c=='x')
   {
     PID_pitch.setD(PID_pitch.getD()-0.02);
     PID_roll.setD(PID_roll.getD()-0.02);
   }
    
  }

 
}




