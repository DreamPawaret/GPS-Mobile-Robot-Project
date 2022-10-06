#include <Wire.h>
#include <ESP8266WiFi.h>

void setup() {
 Serial.begin(9600); /* begin serial for debug */
 Wire.begin(D1, D2); /* join i2c bus with SDA=D1 and SCL=D2 of NodeMCU */
}

float latitude = 7.006595;
char sendLatitude[8];

void loop() {
 Wire.beginTransmission(8); /* begin with device address 8 */
 dtostrf(latitude, 1, 6, sendLatitude);
 Wire.write(sendLatitude);
 Wire.endTransmission();

 delay(1000);
}
