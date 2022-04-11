#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

#include "Wire.h"
#include "SHT31.h"
#define SHT31_ADDRESS   0x44

const char *ssid = "ESP32";
const char *password = "ESP32TTGO";

WiFiServer server(80);
SHT31 sht;

void setup() 
{
  Wire.begin();
  sht.begin(SHT31_ADDRESS);
  
  Serial.begin(115200);
  Serial.println();
  Serial.println("Configuration du point d'accès...");

  WiFi.softAP(ssid, password);
  IPAddress monIP = WiFi.softAPIP();
  
  Serial.print("adresse IP ESP32: ");
  Serial.println(monIP);
  
  server.begin();
  Serial.println("Server started");
}

void loop()
{
        sht.read();
        float t = sht.getTemperature();
        float h = sht.getHumidity();
WiFiClient client = server.available();
if ( client) 
    {

   
      Serial.println("New Client.");
         while (client.connected()){
         
           client.read();
           //client.println("<h1>Station Meteo Connectees</h1>");
           client.println("Station Meteo Connectees");
           client.print("Temp= ");
           client.print(t);
           client.println(" C");
           client.print("Humi= ");
           client.print(h);
           client.println(" %");
           delay (1);
           client.stop();
           //client.print("<meta http-equiv='refresh' content='2'>");
           Serial.println("Client Disconnected.");
         }  
     }
 }
