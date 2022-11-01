#include<math.h>
//#include <Wire.h>
//#include <MechaQMC5883.h>
#define pi 3.14159265358979323846
int is_turnleft = 0;

MechaQMC5883 qmc;
double get_gps_position(){
    
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

double get_distance(double lat1, double lon1, double lat2, double lon2) {
    lat1 = to_radians(lat1);
    lon1 = to_radians(lon1);
    lat2 = to_radians(lat2);
    lon2 = to_radians(lon2);

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double ans = pow(sin(dlat/2),2) + cos(lat1) * cos(lat2) * pow(sin(dlon/2),2);

    ans = 2 * asin(sqrt(ans));

    double world_rarius = 6371;

    ans = ans * world_rarius;

    return ans; //distance between two point
}

//มุมที่รถต้องหมุนไปยังทิศจุดหมาย
double get_rotate_angle(double lat1, double lon1, double lat2, double lon2, double car_angle_ref_n){
    double dlat = lat2 - lat1;
    double dlon = lon2 -lon1;
    double angle_ref_x;

    angle_ref_x = atan(dlat / dlon); //result is radians
    angle_ref_x = to_degree(angle_ref_x); //result is degree
    
    is_turnleft = 0;//-----------------------------------------------------------------------------turn right
    if(dlon>0 && dlat>0){//----------------------------------------------x+ y+ 
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

    is_turnleft = 1; //----------------------------------------------------------------------------turn left
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

int get_azimuth_angle(){
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
    Serial.print("     ");
  
    return azimuth_ang; 

}

void setup() {
    Wire.begin();
    Serial.begin(9600);
    qmc.init();

}

void loop() {
    double lat1 = 18.741318;    //y1
    double lon1 = 98.943172;    //x1
    double lat2 = 18.741928;  //y2
    double lon2 = 98.941476;    //x2

    double car_angle_ref_n = get_azimuth_angle();
    double rotate_ang = get_rotate_angle(lat1, lon1, lat2, lon2, car_angle_ref_n);
    double distance = get_distance(lat1, lon1, lat2, lon2);
    
    Serial.print(" rotate angle : ");
    Serial.print(rotate_ang);
    Serial.print("\t distance : ");
    Serial.print(distance);
    Serial.println(" km");

}