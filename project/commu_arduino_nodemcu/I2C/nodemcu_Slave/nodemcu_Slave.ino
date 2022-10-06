#include <Wire.h>
#include <ESP8266WiFi.h>

void setup() {
 Serial.begin(9600); /* begin serial for debug */
 Wire.begin(D1, D2); /* join i2c bus with SDA=D1 and SCL=D2 of NodeMCU */
 Wire.begin(8);
 Wire.onRequest(requestEvent);
 
}

void loop() {
 delay(1000);
}

void requestEvent(){
 Wire.write("Hello Arduino");  /* sends hello string */

}
