#include<SoftwareSerial.h>
SoftwareSerial ArduinoSerialTransmitter(18, 19);
SoftwareSerial ArduinoSerialReceiver(16, 17);

void setup() {
  Serial.begin(9600);
  ArduinoSerialTransmitter.begin(57600);
  ArduinoSerialReceiver.begin(57600);
}

int num = 0;

void loop() {
    float lat1 = 18.741318;          //y1
    float lon1 = 98.943172;          //x1
    float lat2 = 18.741928;          //y2
    float lon2 = 98.941476;   
    
  while (ArduinoSerialReceiver.available() > 0)
  { num = ArduinoSerialReceiver.parseInt();
    
    if (ArduinoSerialReceiver.read() == '\n')
    {
      Serial.print("Arduino");
      Serial.print(" : ");Serial.println(num);
    }
  }
  
  ArduinoSerialTransmitter.print(lat1);
  ArduinoSerialTransmitter.print(" ");
  ArduinoSerialTransmitter.print(lon1);  
  ArduinoSerialTransmitter.print(" ");
  ArduinoSerialTransmitter.print(lat2);
  ArduinoSerialTransmitter.print(" ");
  ArduinoSerialTransmitter.print(lon2);  
  ArduinoSerialTransmitter.print("\n");
  delay(1000);
}
