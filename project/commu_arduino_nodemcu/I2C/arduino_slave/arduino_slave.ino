#include <Wire.h>

float latitude;
float longitude = 100.502193;
String dataStr = "";
void setup() {
 Wire.begin(8);                /* join i2c bus with address 8 */
 Wire.onReceive(receiveEvent); /* register receive event */
 Serial.begin(9600);           /* start serial for debug */
}
void loop() {
  //Serial.println(longitude, 12);
 delay(1000);
}
// function that executes whenever data is received from master
void receiveEvent() {
 while (Wire.available()) {
    char c = Wire.read(); 
    dataStr = dataStr + c;
    latitude = dataStr.toFloat();
  }
  Serial.print(latitude/1000000, 6);  
  Serial.print("\t"); 
  Serial.println(longitude, 6);
        
}

// function that executes whenever data is received from master
//void receiveEvent(int howMany) {
// while (0 <Wire.available()) {
//    char c = Wire.read();      /* receive byte as a character */
//    Serial.print(c);           /* print the character */
//  }
// Serial.println();             /* to newline */
//}
