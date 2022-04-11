#include "Wire.h"
#include "SHT31.h"
#define SHT31_ADDRESS   0x44
SHT31 sht;



#include <TFT_eSPI.h>
#include <SPI.h>
#define WAIT 2000
TFT_eSPI tft = TFT_eSPI();

void setup() 
{
          Wire.begin();
          sht.begin(SHT31_ADDRESS);

  
          tft.init();
          tft.begin();
          tft.setRotation(1);     
}

void loop() 
{
 sht.read();
 float t = sht.getTemperature();
 float h = sht.getHumidity();

 
          tft.setTextSize(2);
          tft.fillScreen(TFT_BLUE);
          tft.setTextColor(TFT_WHITE, TFT_BLUE);
          tft.drawString(" TEMP       HUMI ", 0, 0, 2);
          tft.drawLine(0, 30, 250, 30, TFT_WHITE);
          tft.drawLine(0, 35, 250, 35, TFT_WHITE);
          tft.setCursor(0, 50);
          tft.setTextSize(2);
          tft.print(t); tft.print(" C");
          tft.setTextSize(2);
          tft.setCursor(160, 50);
          tft.print(h); tft.print(" %");
          while(1) yield();
}
