#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>

SoftwareSerial nodeMCU(D2,D3);

void setup(){
  Serial.begin(9600);
  nodeMCU.begin(4800);
  pinMode(D2,INPUT);
  pinMode(D3,OUTPUT);

}

void loop() {
  while(nodeMCU.available() > 0{
      float val = nodeMCU.parseFloat();
      if(nodeMCU.read() == "\n"){
        Serial.print(val);
      }
      
    }

}
