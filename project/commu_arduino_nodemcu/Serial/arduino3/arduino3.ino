#include <SoftwareSerial.h>

void setup()
{
  Serial.begin(9600);
  Serial1.begin(115200);
}

void loop() {
  float latitude; 
  double longitude;
  static char serialData;
  static byte serialBuffer[4];
  static byte serialIndex = 0;
  static byte serialState = 0;

  while(Serial1.available() > 0){
    serialData = Serial1.read();
    switch(serialState) {
      case 0: if(serialData == '#'){
                serialState = 1;
                serialIndex = 0;
              }
              break;
      case 1: serialBuffer[serialIndex] = serialData;
              if(++serialIndex == 4){
                serialState = 2;
              }
              break;
      case 2: if(serialData == ';'){
                *((int8_t*)(&latitude)+0) = serialBuffer[0];
                *((int8_t*)(&latitude)+1) = serialBuffer[1];
                *((int8_t*)(&latitude)+2) = serialBuffer[2];
                *((int8_t*)(&latitude)+3) = serialBuffer[3];
//                *((int8_t*)(&longitude)+4) = serialBuffer[4];
//                *((int8_t*)(&longitude)+5) = serialBuffer[5];
//                *((int8_t*)(&longitude)+6) = serialBuffer[6];
//                *((int8_t*)(&longitude)+7) = serialBuffer[7];
                Serial.print("latitude : ");
                Serial.println(latitude, 6);
              }
              break;
    }
  }

}
