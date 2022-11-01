#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <Wire.h>
SoftwareSerial NodeSerialReceiver(D3, D4);      // D3(RX)-->19(TX) | D4(TX)-->18(RX)
SoftwareSerial NodeSerialTransmitter(D1, D2);   // D1(RX)-->17(TX) | D2(TX)-->16(RX)

const char* ssid = "username778";
const char* pass = "gift4321";
void setup() {
  
  Serial.begin(9600);
  NodeSerialReceiver.begin(57600);
  NodeSerialTransmitter.begin(57600);
  
  WiFi.begin(ssid, pass);
  
  Serial.println();
  Serial.println("NodeMCU/ESP8266 Run");
  while (WiFi.status() != WL_CONNECTED) { 
    Serial.println("Connecting...  ");  
    Serial.printf("Connection Status: %d\n", WiFi.status()); 
    delay(1000);
  }
  Serial.print("Wi-Fi connected."); 
  Serial.print("IP Address : ");
  Serial.printf("Connection Status: %d\n", WiFi.status());
}

float lat1 = 18.741318;          //y1
float lon1 = 98.943172;          //x1
float lat2 = 18.741928;          //y2
float lon2 = 98.941476; 

void loop() {
  
  while (NodeSerialReceiver.available() > 0)
  {  lat1 = NodeSerialReceiver.parseFloat();
     lon1 = NodeSerialReceiver.parseFloat();
     lat2 = NodeSerialReceiver.parseFloat();
     lon2 = NodeSerialReceiver.parseFloat();
    
    if (NodeSerialReceiver.read() == '\n')
    {
      Serial.print("NodeMCU");
      Serial.print(" : ");Serial.print(lat1,8);
      Serial.print(" : ");Serial.print(lon1,8);
      Serial.print(" : ");Serial.print(lat2);
      Serial.print(" : ");Serial.println(lon2);
    }
  }
  
  NodeSerialTransmitter.print(20);
  NodeSerialTransmitter.print("\n");
  delay(1000);
}
