#include "Wire.h"
#include "DHT.h"
#include "SHT31.h"
#include <rgb_lcd.h>
#define SHT31_ADDRESS   0x44                  //L2C ADRESS 0x44

SHT31 sht;                                    //declaration capteur sht type SHT31
DHT dht(2, DHT22);                            //declaration capteur dht type DHT22 branché sur le port D2 du shield
rgb_lcd lcd;                                  //déclaration d'un écran Lcd

void setup()
{
lcd.begin(16,2);                            //initialisation du LCD en 16 colonnes, 2 lignes
Wire.begin();
      sht.begin(SHT31_ADDRESS);               //l'adresse sht sur le L2C
      dht.begin();                            // Setup du capteur de température/Humidité
}

void loop()
{
float temp_hum_val[1] = {0};                  //tableau qui va accueillir les valeurs de température et humidité  
  dht.readTempAndHumidity(temp_hum_val);      //lecture des valeurs et stockage dans le tableau
    lcd.setCursor(0,0); 
    lcd.print("T1= ");
    lcd.print(temp_hum_val[1]);
    lcd.print(" C");
//*************************************
   sht.read();
     lcd.setCursor(0,1);
     lcd.print("T2= ");
     lcd.print (sht.getTemperature());        //recuperer la valeur du temperature
     lcd.print(" C");
delay(2500);
//*****************************************************************************************************************
  dht.readTempAndHumidity(temp_hum_val);      //lecture des valeurs et stockage dans le tableau
    lcd.setCursor(0,0);                       // Positionnement du curseur en 0,0 (en haut à gauche de l'écran lcd)
    lcd.print("H1= ");                        // Affichage d'une chaine de caractère sur le lcd
    lcd.print(temp_hum_val[0]);
    lcd.print(" %");
//*************************************
     lcd.setCursor(0,1);
     lcd.print("H2= ");
     lcd.print (sht.getHumidity());           //recuperer la valeur du Humidite
     lcd.print(" %");
delay(2500);
}
