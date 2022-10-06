#include <Wire.h>
#include <MechaQMC5883.h>

MechaQMC5883 qmc;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  qmc.init();
  //qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_2G,OSR_256);
}

void loop() {
  int x, y, z;
  int azimuth;
  //float azimuth; //is supporting float too
  qmc.read(&x, &y, &z, &azimuth);
  azimuth = qmc.azimuth(&y,&x);//you can get custom azimuth
  Serial.println();
  Serial.print("x: ");
  Serial.print(x);
  Serial.print(" y: ");
  Serial.print(y);
  Serial.print(" z: ");
  Serial.print(z);
  Serial.print(" a: ");
  Serial.print(azimuth);
  Serial.println();
  if (y > 2060 && x < -1200 && x > -1340) {
    Serial.println("N");
  }
  else if (y < 800 && y > 700 && x < 0 && x > -200) {
    Serial.println("W");
  }
  else if (y < 750 && y > 550 && x < -2500) {
    Serial.println("E");
  }
  else if (y < -450 && y > -650 && x < -1000 && x > -1200) {
    Serial.println("S");
  }
  else{ }
}
