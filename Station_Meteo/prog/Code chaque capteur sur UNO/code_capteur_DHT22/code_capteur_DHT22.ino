#include "DHT.h"
#include <Wire.h>
#include <rgb_lcd.h>

                      
DHT dht(2, DHT22);   //  Définition du type de capteur utiliser pour la librairie
      // on est branché sur le port D2 du shield

rgb_lcd lcd; //déclaration d'un écran Lcd


// Création de motifs carré remplis et carré vide pour le lcd
byte customChar[] = {
  0xFF,
  0xFF,
  0xFF,
  0xFF,
  0xFF,
  0xFF,
  0xFF,
  0xFF
};

byte customChar2[] = {
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00
};


void setup() 
{
  
    
    lcd.begin(16,2); //iitialisation du LCD en 16 colonnes, 2 lignes
    dht.begin(); // Setup du capteur de température/Humidité
    lcd.createChar(1, customChar); // assignation des motifs carré à des numéros ici 1 et 2
    lcd.createChar(2, customChar2);
}

void loop()
{
    float temp_hum_val[2] = {0}; //tableau qui va accueillir les valeurs de température et humidité
    int i = 0;
    dht.readTempAndHumidity(temp_hum_val); //lecture des valeurs et stockage dans le tableau
    lcd.setCursor(0,0); // Positionnement du curseur en 0,0 (en haut à gauche de l'écran lcd)
    
    lcd.print("H1:"); // Affichage d'une chaine de caractère sur le lcd
    lcd.print(temp_hum_val[0]);
    
    lcd.setCursor(0,1);
    lcd.print("H2:");
    lcd.print(temp_hum_val[0]);
      
    for (i = 0; i < 5; i++) // Boucle permettant l'affichage du "chargement"
      {
        lcd.setCursor(i+11, 0); 
        lcd.write(1); // écriture du motif 1 (carré remplis) à la colonne i+11 et ligne 0 de l'écran lcd
        lcd.setCursor(i+11, 1);
        lcd.write(1); // écriture du motif 1 (carré remplis) à la colonne i+11 et ligne 1 de l'écran lcd
        delay(1000); // delais de 1sec
        
      }
    
    lcd.setCursor(0,0); 
    lcd.print("T1:");
    lcd.print(temp_hum_val[1]);
    
    lcd.setCursor(0,1);
    lcd.print("T2:");
    lcd.print(temp_hum_val[1]);
    
    for (i = 0; i < 5; i++)   // Boucle permettant l'affichage du "chargement"
      {
        lcd.setCursor(i+11, 0);
        lcd.write(2);   // écriture du motif 2 (carré vide) à la colonne i+11 et ligne 1 de l'écran lcd
        lcd.setCursor(i+11, 1);
        lcd.write(2);  // écriture du motif 2 (carré vide) à la colonne i+11 et ligne 1 de l'écran lcd
        delay(1000);   // delais de 1sec
      }
    
    
}
