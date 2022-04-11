#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

const char *ssid = "ESP32";
const char *password = "ESP32TTGO";

WiFiServer server(80);

void setup() 
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Configuring access point...");

  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  
  server.begin();
  Serial.println("Server started");
}

void loop()
{
WiFiClient client = server.available();
if (client) 
    {
      Serial.println("New Client.");
          while (client.connected())
          {
           client.read();
           client.println("hello from ESP32 TTGO");
           client.stop();
          }
     }

}
