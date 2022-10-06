/*
 * Rui Santos 
 * Complete Project Details https://randomnerdtutorials.com
 */
 
//#include <SoftwareSerial.h>
//// The serial connection to the GPS module
//SoftwareSerial ss(4, 3);
//
//void setup(){
//  Serial.begin(9600);
//  ss.begin(9600);
//}
//
//void loop(){
//  while (ss.available() > 0){
//    // get the byte data from the GPS
//    byte gpsData = ss.read();
//    Serial.write(gpsData);
//  }
//}

 #include <TinyGPS++.h>
 #include <SoftwareSerial.h>

 static const int RXPin = 4, TXPin = 3;
 static const uint32_t GPSBaud = 9600;

// // The TinyGPS++ object
 TinyGPSPlus gps;

// // The serial connection to the GPS device
 SoftwareSerial ss(RXPin, TXPin);

 void setup(){
   Serial.begin(9600);
   ss.begin(GPSBaud);
 }

 void loop(){
   // This sketch displays information every time a new sentence is correctly encoded.
   while (ss.available() > 0){
     gps.encode(ss.read());
     if (gps.location.isUpdated()){
       Serial.print("Latitude= "); 
       Serial.print(gps.location.lat(), 6);
       Serial.print(" Longitude= "); 
       Serial.println(gps.location.lng(), 6);
     }
   }
 }
