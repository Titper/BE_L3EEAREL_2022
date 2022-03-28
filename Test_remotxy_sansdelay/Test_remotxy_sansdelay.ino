/*
   -- New project --
   
   This source code of graphical user interface 
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 3.1.8 or later version 
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/                   
     - for ANDROID 4.11.1 or later version;
     - for iOS 1.9.1 or later version;
    
   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.    
*/
#include <Wire.h>
//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__SOFTSERIAL
#include <SoftwareSerial.h>

#include <RemoteXY.h>

// RemoteXY connection settings 
#define REMOTEXY_SERIAL_RX 2
#define REMOTEXY_SERIAL_TX 3
#define REMOTEXY_SERIAL_SPEED 9600


// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 19 bytes
  { 255,0,0,11,0,12,0,16,27,0,67,4,8,7,85,10,2,26,11 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // output variables
  char text_1[11];  // string UTF8 end zero 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

int dist(dist_pres)
{
 int visu_trig
 if (((micros()-tmps) > 10) && ( visu_trig == 1))
 {
   digitalWrite(TRIG, LOW);
   digitalWrite(TRIG2, LOW);
   visu_trig = 0;
   tmps = micros();
   return(dist_pres)
 }


  Serial.print("Distance = ");
  Serial.print(dist_pres);
  Serial.println(" cm");
 
}
  
  delayMicroseconds(10);

  
  digitalWrite(TRIG, LOW);
  digitalWrite(TRIG2, LOW);

  //mesure de la distance en cm capteur / cible
  distance = pulseIn(ECHO, HIGH, TIMEOUT) / 58; // formule calcule de la distance (voir documentation capteur
}

void setup() 
{
  int dist_pres
  RemoteXY_Init (); 
  
  
 
  pinMode(TRIG, OUTPUT); // configuration pin TRIG en sortie, elle sera raccordée à l'entrée Trig du capteur HC-SR04
  pinMode(ECHO, INPUT); // configuration pin ECHO en entrée, elle sera raccordée à la sortie Echo du capteur HCSR04
  pinMode(TRIG2, OUTPUT); // mesure 
  
  digitalWrite(TRIG, LOW); //Elle doit être à 0 au démarrage
  digitalWrite(TRIG2, LOW);
  
  Serial.begin(9600);

}

void loop() 
{ 
  RemoteXY_Handler ();
  
  
  // TODO you loop code
  // use the RemoteXY structure for data transfer
  // do not call delay() 


}
