#include <SoftwareSerial.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;

double Lat;
double Long;
int day, month, year;
int hour, minute, second;

int num_sat;

boolean one_point_true = false;

void setup() {
    Serial.begin(9600);

     //GPS connection 
  Serial.println(F("Starting GPS"));
  //GPS com port
  Serial2.begin(9600);

}

void loop() {
   Get_GPS(); // Get the position

}

void Get_GPS()
{

  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))

      num_sat = gps.satellites.value();
  //Serial.println(num_sat);

  if (gps.location.isValid() == 1) {

    Lat = gps.location.lat();
    Long = gps.location.lng();

    if (Lat != 0 && Long != 0) one_point_true = true;

  }




  if (gps.date.isValid())
  {
    day = gps.date.day();
    month = gps.date.month();
    year = gps.date.year();
  }

  if (gps.time.isValid())
  {

    hour = gps.time.hour();
    minute = gps.time.minute();
    second = gps.time.second();
  }

  smartDelay(500);


  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }


}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (Serial2.available())
      gps.encode(Serial2.read());
  }
  while (millis() - start < ms);
}
