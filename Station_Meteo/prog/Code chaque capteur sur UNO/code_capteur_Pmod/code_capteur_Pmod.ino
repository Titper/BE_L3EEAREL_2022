#include <Wire.h>                                                 // call library
#define TMP2_address 0x4B                                         // I2C adress of the Pmod TMP2        
#include <SoftwareSerial.h>
#include <rgb_lcd.h>
rgb_lcd lcd; //declaration


void setup()
{
  lcd.begin (16,2);                                              //ligne colone 2 16
  Serial.begin(9600);                                            // initialization of serial communication
  Wire.begin();                                                  // initialization of I2C communication
  TMP2_init();                                                   // initialisztion of Pmod TMP2 module
}

void loop()
{
// display in serial monitor
  Serial.print("Temperature = ");
  Serial.print(TMP2_read());
  Serial.println("°C");
delay(1000);

//display on LCD 16x2
   lcd.setCursor(0,0);
     lcd.print("TEMP= ");
     lcd.print(TMP2_read());
     lcd.print(" C");
 delay(1000);
}












// Initialization of Pmod TMP2 module
void TMP2_init(void)
{
// Configuring the TMP2 in 16 bit mode
  Wire.beginTransmission(TMP2_address);
  Wire.write(0x03);
  Wire.write(0x80);
  Wire.endTransmission();
}

//measure temperature
float TMP2_read(void) {
// Launch of the measure
  Wire.beginTransmission(TMP2_address);
  Wire.endTransmission();
  delay(10);

// Recovery of the two bytes MSB and LSB
  byte MSB, LSB;
  Wire.requestFrom(TMP2_address, 2);
  if (Wire.available() <= 2)
  {
    MSB = Wire.read();
    LSB = Wire.read();
  }

  //data processing
  int value = (MSB << 8) | LSB ; //reconstruct 2 byte value
  float tmp;
  if (((value >> 15) & 1) == 0) // If the temperature is positive (it is in 2C)
  {
    tmp = value / 128.0;  //just divide it
  }
  else // If the temperature is negative
  {
    tmp = (value - 65535) / 128.0;  //substract the highest number on 16 bts, then divide it
  }
  return tmp;
}
