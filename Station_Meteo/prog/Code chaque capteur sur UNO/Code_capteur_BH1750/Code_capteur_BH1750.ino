#include "Wire.h"
#include "BH1750.h"
#include <rgb_lcd.h>
#define BH1750_ADDRESS   0x23

BH1750 lum; 
rgb_lcd lcd;

void setup()
{
  lcd.begin (16,2);
  Serial.begin(115200);
  Wire.begin();
  lum.begin(BH1750_ADDRESS);
  lum.begin();
}
void loop() 
{
  Serial.print("Light: ");
  Serial.print(lum.readLightLevel());
  Serial.print(" lx");
  Serial.println();

    lcd.setCursor(0,0);
    lcd.print("Lum= ");
    lcd.print(lum.readLightLevel());
    lcd.print(" lx");
  
  delay(2000);
}
