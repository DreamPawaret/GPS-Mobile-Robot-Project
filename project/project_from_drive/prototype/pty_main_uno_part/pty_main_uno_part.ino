#include<math.h>
#include <Wire.h>
#include <MechaQMC5883.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define pi 3.14159265358979323846

int is_turnleft = 0;
static const int GPSRXPin = 4, GPSTXPin = 3;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
MechaQMC5883 qmc;
SoftwareSerial arduinoUno(3,2);

SoftwareSerial ssgps(GPSRXPin, GPSTXPin);

double getLongitude(){
    double lon2;
    while(ssgps.available() > 0){
      gps.encode(ssgps.read());
      {
        if(gps.location.isUpdated()){
          lon2 = gps.location.lng();
        }
      }
    }
    return lon2;
}

double getLattitude(){
    double lat2;
    while(ssgps.available() > 0){
      gps.encode(ssgps.read());
      {
        if(gps.location.isUpdated()){
          lat2 = gps.location.lng();
        }
      }
    }
    return lat2;
}

//เปลี่ยนค่าจากเรียนเดียนเป็นองศา
double to_degree(double radian){
    radian = (radian * 180) / PI;
    return radian;
}

double to_radians( double degree){
    double one_deg = pi / 180;
    return (one_deg * degree);
}

double distance(double lat1, double lon1, double lat2, double lon2) {
    lat1 = to_radians(lat1);
    lon1 = to_radians(lon1);
    lat2 = to_radians(lat2);
    lon2 = to_radians(lon2);

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double a = pow(sin(dlat/2),2) + cos(lat1) * cos(lat2) * pow(sin(dlon/2),2);

    double c = 2 * atan2(sqrt(a), sqrt(1-a));

    double world_rarius = 6371;

    double d = c * world_rarius;

    double ans = d * pow(10,3);

    return ans; //distance between two point
}

//มุมที่รถต้องหมุนไปยังทิศจุดหมาย
double rotate_angle(double lat1, double lon1, double lat2, double lon2, double car_angle_ref_n){
    double dlat = lat2 - lat1;
    double dlon = lon2 -lon1;
    double angle_ref_x;  //มุมที่พิกัดทำกับแกน x 

    angle_ref_x = atan(dlat / dlon); //result is radians
    angle_ref_x = to_degree(angle_ref_x); //result is degree

    Serial.print("dlat");
    Serial.print(dlat);
    
    Serial.print("dlon");
    Serial.print(dlon);
    
    Serial.print(" angle_ref_x : ");
    Serial.print(angle_ref_x);

//    printf("%f\n", angle_ref_x);
    
    //is_turnleft = 0;//-----------------------------------------------------------------------------turn right
    if(dlon>0 && dlat>0){//-----------------------------------------------x+ y+ 
        return 90 - (car_angle_ref_n + angle_ref_x);
    }

    if(dlon>0 && dlat==0){//----------------------------------------------x+ y=0
        return 90 - (car_angle_ref_n + angle_ref_x);
    }

    if(dlon>0 && dlat<0){//----------------------------------------------x+ y-
        return (90 + abs(angle_ref_x)) - car_angle_ref_n;
    }

    if(dlon==0 && dlat<0){//----------------------------------------------x=0 y-
        return 180 - car_angle_ref_n;
    }

    //is_turnleft = 1; //----------------------------------------------------------------------------turn left
    if(dlon<0 && dlat<0){//----------------------------------------------x- y-
        return (270 - angle_ref_x) - car_angle_ref_n;
    }

    if(dlon<0 && dlat==0){//----------------------------------------------x- y=0
        return 270 - car_angle_ref_n;
    }

    if(dlon<0 && dlat>0){//----------------------------------------------x- y+
        return 270 + abs(angle_ref_x) -car_angle_ref_n;
    }

    if(dlon==0 && dlat>0){//----------------------------------------------x=0 y+
        return 360 - car_angle_ref_n;
    }
    
}

int azimuth_angle(){
    int x, y, z;
    int azimuth_ang;
  
    qmc.read(&x, &y, &z, &azimuth_ang);
  
    azimuth_ang = qmc.azimuth(&y,&x);
  
    Serial.println();
    Serial.print("x: ");
    Serial.print(x);
    Serial.print(" y: ");
    Serial.print(y);
    Serial.print(" z: ");
    Serial.print(z);
    Serial.print(" azimuth: ");
    Serial.print(azimuth_ang);
  
    return azimuth_ang;
    
}

void setup() {
    Wire.begin();
    Serial.begin(9600);
    ssgps.begin(GPSBaud);
    arduinoUno.begin(4800);
    qmc.init();

}

void loop() {
    double lat1 = 7.011044;          //y1
    double lon1 = 100.502519;          //x1
    double lat2 = 7.011181;     //y2
    double lon2 = 100.501883;     //x2
//    double lat1 = 0;          //y1
//    double lon1 = 0;          //x1
//    double lat2 = 100;     //y2
//    double lon2 = 0;     //x2

    //======================================================================================
    //azimuth_angle();
    double car_angle_ref_n = azimuth_angle();
    double rotate_ang = rotate_angle(lat1, lon1, lat2, lon2, car_angle_ref_n);
    double dis = distance( lat1, lon1, lat2, lon2);
    
    Serial.print(" rotate_angle : ");
    Serial.print(rotate_ang);
//    float i = rotate_ang;
//    float j = distance(lat1, lon1, lat2, lon2);
//    Serial.print(" azimuth: ");
//    Serial.print(car_angle_ref_n);
//    Serial.print("     ");
//    Serial.print(" rotate angle : ");
//    arduinoUno.print(i);
    Serial.print("\t distance : ");
    Serial.println(dis);
//    arduinoUno.print(j);
//    Serial.println(" m");
    
    
//    Serial.print(" rotate angle : ");
//    Serial.print(rotate_ang);
//    Serial.print("\t distance : ");
//    Serial.print(distance(lat1, lon1, lat2, lon2));
//    Serial.println(" m");

}
