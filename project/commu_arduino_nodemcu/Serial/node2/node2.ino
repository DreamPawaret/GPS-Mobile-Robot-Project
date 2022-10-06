#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
SoftwareSerial NodeSerial(D2, D3); // RX | TX

int num1 = 1234;

void setup() {
  pinMode(D2, INPUT);
  pinMode(D3, OUTPUT);
  Serial.begin(9600);
  NodeSerial.begin(57600);
  Serial.println();
  Serial.println("NodeMCU/ESP8266 Run");
}

void loop() {
  NodeSerial.print(num1);
  NodeSerial.print("\n");
  delay(1000);
//  while (NodeSerial.available() > 0)
//  {
//    float i_data = NodeSerial.parseFloat();
//    float f_data = NodeSerial.parseFloat();
//    if (NodeSerial.read() == '\n')
//    {
//      Serial.print("NodeMCU or ESP8266");
//      Serial.print(" : ");
//      Serial.print(i_data); Serial.print(" : ");
//      Serial.println(f_data);
//    }
//  }

}
