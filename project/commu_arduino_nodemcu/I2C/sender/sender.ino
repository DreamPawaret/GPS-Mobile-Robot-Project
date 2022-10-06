#include <Wire.h>

int slave_address = 8;

float latitude;
char sendLatitide;

void setup() {
  Wire.begin(slave_address);
  Wire.onRequest(requestEvent);

}

void loop() {

}

void requestEvent(){
  Wire.write("Hello ");
}
