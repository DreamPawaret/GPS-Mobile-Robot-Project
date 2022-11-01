#define BLYNK_TEMPLATE_ID "TMPL_4KB0G69"
#define BLYNK_DEVICE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "HYE4e0RetGJKePtA3d1CS-53S8yilb-q"

// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>
#include <Wire.h>

SoftwareSerial NodeSerialReceiver(D3, D4);      // D3(RX)-->19(TX) | D4(TX)-->18(RX)
SoftwareSerial NodeSerialTransmitter(D1, D2);   // D1(RX)-->17(TX) | D2(TX)-->16(RX)

char auth[] = BLYNK_AUTH_TOKEN;

char ssid[] = "username778";
char pass[] = "gift4321";

BlynkTimer timer;

BLYNK_WRITE(V0)
{
  int value = param.asInt();
  digitalWrite(D4, !value);
  Blynk.virtualWrite(V1, value);
}

BLYNK_WRITE(V4){
  double pinValue = param.asDouble();
    //Blynk.virtualWrite(V6, pinValue);
}

BLYNK_WRITE(V5){
  double pinValue = param.asDouble();
    Blynk.virtualWrite(V7, pinValue);
}

BLYNK_WRITE(V6)
{
  int value = param.asDouble();
}


BLYNK_WRITE(V7)
{
  double value = param.asDouble();
}

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
  // Change Web Link Button message to "Congratulations!"
  Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
  Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
  Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
}

// This function sends Arduino's uptime every second to Virtual Pin 2.
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V2, millis() / 1000);
}

void setup()
{
  pinMode(D4, OUTPUT);
  Serial.begin(115200);
  NodeSerialReceiver.begin(57600);
  NodeSerialTransmitter.begin(57600);

  Blynk.begin(auth, ssid, pass);

  // Setup a function to be called every second
  timer.setInterval(1000L, myTimerEvent);
  Serial.println();
  Serial.println("NodeMCU/ESP8266 Run");
  while (WiFi.status() != WL_CONNECTED) { 
    Serial.println("Connecting...  ");  
    Serial.printf("Connection Status: %d\n", WiFi.status()); 
    delay(1000);
  }
  Serial.print("Wi-Fi connected."); 
  Serial.print("IP Address : ");
  Serial.printf("Connection Status: %d\n", WiFi.status());
}

float lat1;
float lon1;
float lat2;
float lon2;
float sum = 12.345678;
void loop()
{
  while (NodeSerialReceiver.available() > 0)
  {  lat1 = NodeSerialReceiver.parseFloat();
     lon1 = NodeSerialReceiver.parseFloat();
     lat2 = NodeSerialReceiver.parseFloat();
     lon2 = NodeSerialReceiver.parseFloat();
    
    if (NodeSerialReceiver.read() == '\n')
    {
      Serial.print("NodeMCU");
      Serial.print(" : ");Serial.print(lat1, 6);
      Serial.print(" : ");Serial.print(lon1, 6);
      Serial.print(" : ");Serial.print(lat2, 6);
      Serial.print(" : ");Serial.print(lon2, 6);
      Serial.print(" : ");Serial.println(sum, 6);
    }
  }
//  NodeSerialTransmitter.print(20);
//  NodeSerialTransmitter.print("\n");
//  delay(200);
  
  Blynk.virtualWrite(V6, lat2,6);
  Blynk.virtualWrite(V7, lon2,6);
  Blynk.virtualWrite(V9, "123");
  Blynk.virtualWrite(V10, "12345");
  Blynk.run();
  timer.run();
  // You can inject your own code or combine it with other ske?tches.
  // Check other examples on how to communicate with Blynk. Remember
  // to avoid delay() function!
}
