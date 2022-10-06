#define BLYNK_TEMPLATE_ID "TMPLe-37NkZz"
#define BLYNK_DEVICE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "L9CvTeOTPr4W--sZ5lcwbrbkh-jGmeQm"

// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>

SoftwareSerial NodeSerial(D2, D1);  //D2, D1 = SRX, STX

char auth[] = BLYNK_AUTH_TOKEN;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "username778";
char pass[] = "gift4321";

String dataInLatitude1;
String dataInLongitude1;
String dataInLatitude2;
String dataInLongitude2;

float latitude1 = 0.00;
float longitude1 = 0.00 ;

float lat1;
float lon1;

BlynkTimer timer;

BLYNK_WRITE(V1) {
  dataInLatitude1 = param.asStr();  // Text Input Widget - Strings
  Blynk.virtualWrite(V5, dataInLatitude1);
  latitude1 = dataInLatitude1.toFloat();
  lat1 = latitude1 - 7;
   
  //Serial.println(lat1*1000000);
}

BLYNK_WRITE(V2) {
  dataInLongitude1 = param.asStr();  // Text Input Widget - Strings
  Blynk.virtualWrite(V6, dataInLongitude1);
  longitude1 = dataInLongitude1.toFloat();
  lon1 = longitude1 - 100;
  
//  Serial.println(dataInLongitude1); 
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
  Blynk.virtualWrite(V0, random(0, 100));
}

void setup()
{
  // Debug console
  Serial.begin(9600);
  NodeSerial.begin(115200);

  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);

  // Setup a function to be called every second
  timer.setInterval(1000L, myTimerEvent);
}

void loop()
{
//  latitude1 = dataInLatitude1.toFloat();
//  longitude1 = dataInLongitude1.toFloat();
  
//  float lat1 = latitude1 - 7;
//  float lon1 = longitude1 - 100;

//  Serial.print(latitude1, 6);  
//  Serial.print(" : ");
//  Serial.print(longitude1, 6);
//  Serial.print("\t");
  Serial.print(lat1*1000000);  
  Serial.print(" : ");
  Serial.println(lon1*1000000);

  NodeSerial.print(lat1 * 1000000);    
  NodeSerial.print(" ");
  NodeSerial.print(lon1 * 1000000);   
  NodeSerial.print("\n");
  
  Blynk.run();
  timer.run();
  delay(500);
  // You can inject your own code or combine it with other sketches.
  // Check other examples on how to communicate with Blynk. Remember
  // to avoid delay() function!
}
