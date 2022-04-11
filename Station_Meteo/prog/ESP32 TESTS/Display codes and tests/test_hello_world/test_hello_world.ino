#include <TFT_eSPI.h>
#include <SPI.h>
//# include <Free_Fonts.h>                                     // Inclut le fichier d'en-tête attaché à ce croquis
#define FONT &0rbitron_Light_24

TFT_eSPI tft = TFT_eSPI();

void setup() {
  tft.init();
  tft.begin();
  tft.setRotation(1);
}

void loop() {

  int xpos = 20;
  int ypos = 60;
  tft.fillScreen(TFT_BLACK);                                   // Select different fonts to draw on screen using the print class
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setTextSize(3);
  tft.setCursor(xpos, ypos);                                   // Positionne le curseur près du coin supérieur gauche de l'écran
//tft.setFreeFont(&0rbitron_Light_24);                         // Sélectionnez la petite police TomThumb d'origine
  tft.print("HELLO WORLD");
  while(1) yield();                                            // We must yield() to stop a watchdog timeout.
}
