#include "Wire.h"
#include "Dps310.h"
#include <rgb_lcd.h>
#define Dps310_ADDRESS   0x77

Dps310 Pression = Dps310();
rgb_lcd lcd;

void setup()
{
  lcd.begin (16,2);
  Serial.begin(115200);
  Wire.begin();
  Pression.begin(Wire,Dps310_ADDRESS);
}

void loop()
{
  float pres,p;
   p = Pression.measurePressureOnce(pres);
   Serial.print("Pression atmosphérique = ");
   Serial.print(pres/100);
   Serial.print(" Hpa");
   Serial.println();

    lcd.setCursor(0,0);
    lcd.print("Pres=");
    lcd.print(pres/100);
    lcd.print("hPa");
    
delay(1000);
}
