#include <SoftwareSerial.h>

#include <TinyGPS++.h>

static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

void setup(){
 Serial.begin(9600);
 Serial2.begin(9600);
//  ss.begin(GPSBaud);
}

void loop(){
 while (Serial2.available() > 0){
   // get the byte data from the GPS
   gps.encode(Serial2.read());
   if (gps.location.isUpdated()){
     Serial.print("Latitude= "); 
     Serial.print(gps.location.lat(), 6);
     Serial.print(" Longitude= "); 
     Serial.println(gps.location.lng(), 6);
   }
 }
}
