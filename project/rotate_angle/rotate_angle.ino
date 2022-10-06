#include<math.h>
#include <Wire.h>
#include <MechaQMC5883.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const uint32_t GPSBaud = 9600;

//gps module
TinyGPSPlus gps;
//digital compass
MechaQMC5883 qmc;

double toDegree(double radian){
    radian = (radian * 180) / PI;
    return radian;
}

double toRadians( double degree){
    double one_deg = PI / 180;
    return (one_deg * degree);
}

int getCompassHeading(){
    int x, y, z;
    int robot_car_heading;
  
    qmc.read(&x, &y, &z, &robot_car_heading);
  
    robot_car_heading = qmc.azimuth(&y,&x);
  
    return robot_car_heading;  
}

double computeDistance(double waypoint_lat, double waypoint_lon, double robot_car_lat, double robot_car_lon) {
    waypoint_lat = toRadians(waypoint_lat);
    waypoint_lon = toRadians(waypoint_lon);
    robot_car_lat = toRadians(robot_car_lat);
    robot_car_lon = toRadians(robot_car_lon);

    double dlat = robot_car_lat - waypoint_lat;
    double dlon = robot_car_lon - waypoint_lon;
    double a = pow(sin(dlat/2),2) + cos(waypoint_lat) * cos(robot_car_lat) * pow(sin(dlon/2),2);

    double c = 2 * atan2(sqrt(a), sqrt(1-a));

    double world_rarius = 6371.0;

    double d = c * world_rarius;

    double ans = d * pow(10,3);

    return ans; //distance between two point (m)
}

//double computeAngle(double robot_car_lat, double robot_car_lon, double waypoint_lat, double waypoint_lon, double robot_car_heading){
//    double lat2 = toRadians(waypoint_lat);
//    double lon2 = toRadians(waypoint_lon);
//    double lat1 = toRadians(robot_car_lat);
//    double lon1 = toRadians(robot_car_lon);
//    
//    double dlat = lat2 - lat1;
//    double dlon = lon2 - lon1;
//    double angle_ref_x;
//    
////    double waypoint_angle = atan2(sin(dlon) * cos(robot_car_lat) , ( cos(waypoint_lat) * sin(robot_car_lat) ) - ( sin(waypoint_lat) * cos(robot_car_lat) * cos(dlon) ));
////    waypoint_angle = waypoint_angle * 180/PI;    
////    return waypoint_angle;
//    
//
//    angle_ref_x = atan(dlat / dlon); //result is radians
//    angle_ref_x = toDegree(angle_ref_x); //result is degree
//    
//    //is_turnleft = 0;//-----------------------------------------------------------------------------turn right
//    if(dlon>0 && dlat>0){//-----------------------------------------------x+ y+ 
//        return 90 - (robot_car_heading + angle_ref_x);
//    }
//
//    if(dlon>0 && dlat==0){//----------------------------------------------x+ y=0
//        return 90 - (robot_car_heading + angle_ref_x);
//    }
//
//    if(dlon>0 && dlat<0){//----------------------------------------------x+ y-
//        return (90 + abs(angle_ref_x)) - robot_car_heading;
//    }
//
//    if(dlon==0 && dlat<0){//----------------------------------------------x=0 y-
//        return 180 - robot_car_heading;
//    }
//
//    //is_turnleft = 1; //----------------------------------------------------------------------------turn left
//    if(dlon<0 && dlat<0){//----------------------------------------------x- y-
//        return (270 - angle_ref_x) - robot_car_heading;
//    }
//
//    if(dlon<0 && dlat==0){//----------------------------------------------x- y=0
//        return 270 - robot_car_heading;
//    }
//
//    if(dlon<0 && dlat>0){//----------------------------------------------x- y+
//        return 270 + abs(angle_ref_x) - robot_car_heading;
//    }
//
//    if(dlon==0 && dlat>0){//----------------------------------------------x=0 y+
//        return 360 - robot_car_heading;
//    }  
//}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  //Serial2.begin(GPSBaud);
  qmc.init();

}

void loop() {

    double waypoint_latitude = 7.007255;
    double waypoint_longitude = 100.502468;  
    double robot_car_latitude = 7.007132;
    double robot_car_longitude = 100.502289;

    double world_radius = 6371.0;
    

    
//===========================================================================
    waypoint_latitude = toRadians(waypoint_latitude);
    waypoint_longitude = toRadians(waypoint_longitude);
    robot_car_latitude = toRadians(robot_car_latitude);
    robot_car_longitude = toRadians(robot_car_longitude);
    double dlat = waypoint_latitude - robot_car_latitude;
    double dlon = waypoint_longitude - robot_car_longitude;

    double a = pow(sin(dlat / 2),2) + cos(robot_car_latitude) * cos(waypoint_latitude) *  pow( sin(dlon / 2) , 2);
    double d = 2 * world_radius * atan2(sqrt(a), sqrt(1-a));

    int robot_car_heading = getCompassHeading();
    Serial.print("Heading : ");
    Serial.print(robot_car_heading);

    Serial.print("\tdistance : ");
    Serial.print(d*1000);
    Serial.print("m");

//===========================================================================

    double azimuth_angle = atan2( sin(dlon)*cos(waypoint_latitude), (cos(robot_car_latitude) * sin(waypoint_latitude)) - (sin(robot_car_latitude) * cos(waypoint_latitude) * cos(dlon)));
    int azimuth = toDegree(azimuth_angle);
    Serial.print("\tazimuth : ");
    Serial.println(azimuth);
    
    

}
