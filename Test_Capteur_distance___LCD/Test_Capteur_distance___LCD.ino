////////////Déclaration LCD////////////////
//test

#include <Wire.h>
#include "rgb_lcd.h"

rgb_lcd lcd;

////////////Déclaration Capteur Distance //////////////
#define TRIG 12 //pin 12 carte UNO pour envoyer pulse de 10µs
#define ECHO 11 // pin 11 carte UNO pour recevoir pulse Echo
#define TRIG2 13 //recopie pour mesure 
const long TIMEOUT = 30000UL; // 30ms soit mesure à moins de 6m25
unsigned long distance; //
int trig;

////////////////////////////////////////////////////////

void setup() {
  pinMode(TRIG, OUTPUT); // configuration pin TRIG en sortie, elle sera raccordée à l'entrée Trig du capteur HC-SR04
  pinMode(ECHO, INPUT); // configuration pin ECHO en entrée, elle sera raccordée à la sortie Echo du capteur HCSR04
  pinMode(TRIG2, OUTPUT); // mesure 
  
  digitalWrite(TRIG, LOW); //Elle doit être à 0 au démarrage
  digitalWrite(TRIG2, LOW);
  
  Serial.begin(9600);
  
  lcd.begin(16, 2);

  lcd.setCursor(0, 0);
  lcd.print("TRIG = ");

  lcd.setCursor(0, 1);
  lcd.print("DIST = ");

  lcd.setCursor(11,1);
  lcd.print("cm");
}



void loop() {

  //création impulsion trigger de 10µs
  

  
  digitalWrite(TRIG, HIGH);
  digitalWrite(TRIG2, HIGH);
  
  trig = 5;
  Serial.print("TRIG = ");
  Serial.println(trig);
  lcd.setCursor(7, 0);
  lcd.print(trig);

  delayMicroseconds(10);
  
  trig = 0;
  Serial.print("TRIG = ");
  Serial.println(trig);  
  lcd.setCursor(7, 0);
  lcd.print(trig);
   
  digitalWrite(TRIG, LOW);
  digitalWrite(TRIG2, LOW);

  //mesure de la distance en cm capteur / cible
  distance = pulseIn(ECHO, HIGH, TIMEOUT) / 58; // formule calcule de la distance (voir documentation capteur
  
  Serial.print(",");
  //Affichage mesure toutes les 0.5 seconde
  Serial.print("Distance = ");
  Serial.print(distance);
  Serial.println(" cm");
  lcd.setCursor(7, 1);
  lcd.print("    ");
  lcd.setCursor(7, 1);
  lcd.print(distance);
  
  
  delay(100);

}
