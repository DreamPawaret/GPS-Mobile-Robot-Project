#define BLYNK_TEMPLATE_ID "TMPL_4KB0G69"
#define BLYNK_DEVICE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "HYE4e0RetGJKePtA3d1CS-53S8yilb-q"

// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

char auth[] = BLYNK_AUTH_TOKEN;

char ssid[] = "username778";
char pass[] = "gift4321";

static const int RXPin = 4;
static const int TXPin = 3;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

BlynkTimer timer;

BLYNK_WRITE(V0)
{
  int value = param.asInt();
  digitalWrite(D4, !value);

  Blynk.virtualWrite(V1, value);
}

BLYNK_WRITE(V4){
  double pinValue = param.asDouble();
    Blynk.virtualWrite(V6, pinValue);
}

BLYNK_WRITE(V5){
  double pinValue = param.asDouble();
    Blynk.virtualWrite(V7, pinValue);
}

BLYNK_WRITE(V6)
{
  double value = param.asDouble();
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
  Serial.begin(9600);
  ss.begin(GPSBaud);
  Blynk.begin(auth, ssid, pass);

  // Setup a function to be called every second
  timer.setInterval(1000L, myTimerEvent);
}

void loop()
{
  while (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){
      Serial.print("Latitude= "); 
      Serial.print(gps.location.lat(), 6);
//      Blynk.virtualWrite(V6, gps.location.lat());
      Serial.print(" Longitude= "); 
      Serial.println(gps.location.lng(), 6);
//      Blynk.virtualWrite(V7, gps.location.lng());
    }
  }
  Serial.println("hello");
  Blynk.virtualWrite(V9, "123");
  Blynk.virtualWrite(V10, "12345");
  Blynk.run();
  timer.run();
  // You can inject your own code or combine it with other sketches.
  // Check other examples on how to communicate with Blynk. Remember
  // to avoid delay() function!
}
