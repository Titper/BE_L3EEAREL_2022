#include <TFT_eSPI.h>                               // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#define WAIT 2000                                   // Pause in milliseconds between screens, change to 0 to time font rendering
#define TFT_GREY 0x5AEB                             // New colour
TFT_eSPI tft = TFT_eSPI();                          // Invoke library, pins defined in User_Setup.h

void setup() 
{
          tft.init();
          tft.begin();
          tft.setRotation(0);                       //rotation de l'ecran
}

void loop() 
{
          int xpos = 0;
          int ypos = 0;
          tft.fillScreen(TFT_ORANGE);                  //Couleur background
          tft.setTextColor(TFT_WHITE, TFT_ORANGE);     //Couleur Text et fond Text
          tft.drawString("HI Yosri", 0, 0, 4);         //Text à ecrire: (" ++text++ ", x, y, s:taille)
                                                       //Retour à a linge avec y=+16 POUR s=2 y=26 POUR s=4(ex: y=0 ligne1 y=16 ligne2 y=32 ligne3...y=224 ligne14 pour s=2)
          //tft.setTextFont(8);
          delay(WAIT);                              
          tft.fillScreen(TFT_GREY);
          tft.setTextColor(TFT_WHITE, TFT_GREY);
          tft.drawString("HI Vincent", 0, 208, 4);
          //tft.setTextFont(4);
          delay(WAIT);

          
// autre technique pour affichier des caracteres:
          /* 
           int xpos = 0;
           int ypos = 0;
           tft.setTextSize(1);
           tft.fillScreen(TFT_BLACK);
           tft.setTextColor(TFT_GREEN, TFT_BLACK);
           tft.setCursor(xpos, ypos);  
           tft.print("Hi Yosri");
           delay(WAIT);
           //while(1) yield(); // We must yield() to stop a watchdog timeout.
           */
}
