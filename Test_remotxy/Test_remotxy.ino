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



////////////Déclaration Capteur Distance //////////////
#define TRIG 12 //pin 12 carte UNO pour envoyer pulse de 10µs
#define ECHO 11 // pin 11 carte UNO pour recevoir pulse Echo
#define TRIG2 13 //recopie pour mesure 
const long TIMEOUT = 30000UL; // 30ms soit mesure à moins de 6m25
unsigned long distance; //


void setup() 
{
  RemoteXY_Init (); 
  
  
 
  pinMode(TRIG, OUTPUT); // configuration pin TRIG en sortie, elle sera raccordée à l'entrée Trig du capteur HC-SR04
  pinMode(ECHO, INPUT); // configuration pin ECHO en entrée, elle sera raccordée à la sortie Echo du capteur HCSR04
  pinMode(TRIG2, OUTPUT); // mesure 
  
  digitalWrite(TRIG, LOW); //Elle doit être à 0 au démarrage
  digitalWrite(TRIG2, LOW);
  
  Serial.begin(9600);
  sprintf (RemoteXY.text_1,"%05d", "000cm");

}

void loop() 
{ 
  char chaine [11];
  RemoteXY_Handler ();
  
  

  
  digitalWrite(TRIG, HIGH);
  digitalWrite(TRIG2, HIGH);
  
  delayMicroseconds(10);
  
  digitalWrite(TRIG, LOW);
  digitalWrite(TRIG2, LOW);

  //mesure de la distance en cm capteur / cible
  distance = pulseIn(ECHO, HIGH, TIMEOUT) / 58; // formule calcule de la distance (voir documentation capteur
  sprintf (RemoteXY.text_1,"%03d cm", distance);
  delay(100);


}
