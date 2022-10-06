#include <SoftwareSerial.h>

void setup()
{
  Serial.begin(9600);
  Serial3.begin(115200);
}

void loop()
{ 
//  Serial.println("hello");
  while(Serial3.available() > 0){

    float val1 = Serial3.parseFloat();
    float val2 = Serial3.parseFloat();
    
      Serial.print(val1);
      Serial.print(" : ");
      Serial.print(val2);
      Serial.print("\t");

    float lat1;
    float lon1;  
    float latitude;
    float longitude;

    if(val1>=1000000 && val1<2000000 && val2 >= 2000000){
      lat1 = val1 - 1000000;
      latitude = 7 + (lat1/1000000);
      lon1 = val2 - 2000000;
      longitude = 100 + (lon1/1000000) ;
      
    }

    if(val2>=1000000 && val2<2000000 && val1 >= 2000000 && val1<3000000){
              lat1 = val1 - 2000000;
              latitude = 100 + (lat1/1000000);
              
              lon1 = val2 - 1000000;
              longitude = 7 + (lon1/1000000) ;

              float str_coo;
              str_coo = latitude;
              latitude = longitude;
              longitude = str_coo;
              
    } 
   
//    if(Serial3.read() == '\n'){

      Serial.print("waypoint latitude : ");
      Serial.print(latitude, 6);
      Serial.print(" ");
      Serial.print("waypoint longitude : ");
      Serial.println(longitude, 6);

//    }
  }


//    if(c == '\n'){
//      Parse_the_Data();
//
//      Serial.print("latitude : " + data1);
//      Serial.println("longitude : " + data2);
//
//      //reser the variable
//      c = 0;
//      dataIn = " ";
//    }
}
