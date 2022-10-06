
/*
 * Rui Santos 
 * Complete Project Details https://randomnerdtutorials.com
 */

 #include <SoftwareSerial.h>

// The serial connection to the GPS module
//SoftwareSerial ss(4, 3);

void setup(){
 Serial.begin(9600);
 Serial2.begin(9600);
}

void loop(){
 while (Serial2.available() > 0){
   // get the byte data from the GPS
   byte gpsData = Serial2.read();
   Serial.write(gpsData);
   
 }
}
