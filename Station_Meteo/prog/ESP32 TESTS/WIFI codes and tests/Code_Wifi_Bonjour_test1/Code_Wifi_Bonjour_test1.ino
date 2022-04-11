#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>

//const char* ssid = "AndroidAP8CE7";
//const char* password = "moulette";
const char* ssid = "Yosri's Galaxy S20 FE 5G";
const char* password = "ildr7404";
WebServer server(80);

void handleRoot() {
server.send(200, "text/plain", "hello from ESP32 TTGO T-Display <3");
}

void setup(void) {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp32")) {
     Serial.println("MDNS responder started");
  }
  server.on("/", handleRoot);
  
  server.begin();
  Serial.println("HTTP server started");
}

void loop(void) {
  server.handleClient();
  delay(2);//allow the cpu to switch to other tasks
}
