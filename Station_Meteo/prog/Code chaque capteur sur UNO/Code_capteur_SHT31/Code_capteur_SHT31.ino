#include "Wire.h"
#include "SHT31.h"
#include <rgb_lcd.h>              //Affichage sur LCD
#define SHT31_ADDRESS   0x44      //L2C ADRESS 0x44

SHT31 sht;                        //declaration capteur sht type SHT31
rgb_lcd lcd; //declaration

void setup()
{
  lcd.begin (16,2);               //ligne colone 2 16
  Serial.begin(115200);           //code pour moniteur serie
  Wire.begin();
  sht.begin(SHT31_ADDRESS);       //l'adresse sht sur le L2C
}
void loop()
{
   sht.read();

   Serial.print("temp=");
   Serial.print(sht.getTemperature());
   Serial.print(" C ");

    lcd.setCursor(0,0);
    lcd.print("Temp= ");
    lcd.print(sht.getTemperature()); //recuperer la valeur du temperature
    lcd.print(" C");
 
   Serial.print("Humi= ");
   Serial.print(sht.getHumidity());
   Serial.println(" %");
 
     lcd.setCursor(0,1);
     lcd.print("Humi= ");
     lcd.print (sht.getHumidity()); //recuperer la valeur du Humidite
     lcd.print(" %");
delay(1000);
}
