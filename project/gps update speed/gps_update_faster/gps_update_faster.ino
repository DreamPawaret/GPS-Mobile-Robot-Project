#include <SoftwareSerial.h>

#include <TinyGPS++.h>

static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

void sendPacket(byte *packet, byte len) {
 for (byte i = 0; i < len; i++)
 {
 Serial2.write(packet[i]); // GPS is HardwareSerial
 }
}

void changeFrequency() {
    byte packet[] = {
        0xB5, // 
        0x62, // 
        0x06, // 
        0x08, // 
        0x06, // length
        0x00, // 
        0xC8, // measRate, hex 64 = dec 100 ms, C8 = 200 ms, 1FE = 500ms, 3E8 = 1000ms
        0x00, // 
        0x01, // navRate, always =1
        0x00, // 
        0x01, // timeRef, stick to GPS time (=1)
        0x00, // 
        0x7A, // CK_A
        0x12, // CK_B
    };
    sendPacket(packet, sizeof(packet));
}

void setup(){
 Serial.begin(115200);
 Serial2.begin(9600);
  changeFrequency();
  delay(200); 
  Serial2.flush();
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
