

void lecture_capteur_et_conversion()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  accel_lin_x = ax;
  accel_lin_y = ay;
  accel_lin_z = az;
  vit_rot_x = (float)(gx-offset_x)*2000.0f/32768.0f;
  vit_rot_y = (float)(gy-offset_y)*2000.0f/32768.0f;
  vit_rot_z = (float)(gz-offset_z)*2000.0f/32768.0f;
 
}

void filtre_complementaire()
{
  float pitchAcc, rollAcc;

  // L'integration des valeurs de vitesse angulaire du gyroscope nous donne une valeur d'angle
  pitch += vit_rot_x * G_Dt; // Angle autour de l'axe X
  roll -= vit_rot_y * G_Dt; // Angle autour de l'axe Y


  //Pour compenser la dérive gyroscopique on va pondérer les valeurs du gyroscope avec celle de l'accelerometre,
  //à condition que les valeurs de l'accelerometre ne soit pas trop bruitées c'est à dire l'acceleration totale comprise entre 0.75G et 1.25G.
  int acceleration_totale = abs(accel_lin_x) + abs(accel_lin_y) + abs(accel_lin_z);

  if (acceleration_totale > 6144 && acceleration_totale < 10240)
  {
    //Axe X
    pitchAcc = atan2((float)accel_lin_y, (float)accel_lin_z) * 180 / M_PI;
    pitch = pitch * 0.98 + pitchAcc * 0.02;

    //Axe Y
    rollAcc = atan2((float)accel_lin_x, (float)accel_lin_z) * 180 / M_PI;
    roll = roll * 0.98 + rollAcc * 0.02;
  }
}



void calibration_gyroscope()
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
  
  offset_x = gx;
  offset_y = gy;
  offset_z = gz;

  for(int i=0;i<100;i++)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    offset_x = offset_x*0.80 + gx*0.20;
    offset_y = offset_y*0.80 + gy*0.20;
    offset_z = offset_z*0.80 + gz*0.20;
    delay(20);
  }
}


void communication_serie()
{
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print(roll);
  Serial.println(" ");
  
  while(Serial.available())
  {
    char c = Serial.read();
    
    if (c=='a'){kp+=0.02;} 
    else if (c=='z'){kp-=0.02;}
    else if (c=='q'){ki+=0.02;}
    else if (c=='s'){ki-=0.02;}
    else if (c=='w') {kd+=0.01;}
    else if (c=='x') {kd-=0.01;}  
  }
}




