//#include<math.h>
//#include <Wire.h>
//#include <MechaQMC5883.h>
//#include <TinyGPS++.h>
//#include <SoftwareSerial.h>
//
////#define pi 3.14159265358979323846
////==================== motor A (right)
//#define R_PWM_A 4      //IN1
//#define L_PWM_A 5      //IN2
//#define R_EN_A 22
//#define L_EN_A 23
////==================== motor B (left)
//#define R_PWM_B 6      //IN3
//#define L_PWM_B 7      //IN4
//#define R_EN_B 24
//#define L_EN_B 25
//
//int PWM = 100;
//int PWMSLOW = 70; 
//
//int is_turnleft = 0;
//static const uint32_t GPSBaud = 9600;
//
////gps module
//TinyGPSPlus gps;
////digital compass
//MechaQMC5883 qmc;
//
//struct t_waypoint{
//  float latitude;
//  float longitude;
//};
//
//struct robot_car_coordinates{
//  float latitude;
//  float longitude;
//};
//
////============ navigation control ==============
//void forward() {
//  Serial.println("\tForward");
//  motorA_Forward();
//  motorB_Forward();
//}
//
//void backward(int time_val) {
//  Serial.println("\tBackward");
//  motorA_Backward();
//  motorB_Backward();
//}
//
//void turnRight() {
//  Serial.println("\tTurn Right");
//  motorA_Backward();
//  motorB_Forward();
//}
//
//void turnLeft() {
//  Serial.println("\tTurn Left");
//  motorA_Forward();
//  motorB_Backward();
//}
//
//void turnRight_Slow() {
//  Serial.println("\tTurn Right Slow");
//  motorA_Backward_Slow();
//  motorB_Forward_Slow();
//}
//
//void turnLeft_Slow() {
//  Serial.println("\tTurn Left Slow");
//  motorA_Forward_Slow();
//  motorB_Backward_Slow();
//}
//
//void brake() {
//  Serial.println("\tBrake");
//  motorA_Stop();
//  motorB_Stop();
//}
//
////void stop_motor() {
////  Serial.print("Motor stop");
////  motorA_Stop();
////  motorB_Stop();
////}
//
////==================== motor A (right) control
//void motorA_Forward() {
//  analogWrite(R_PWM_A, 0);
//  analogWrite(L_PWM_A, PWM);
//}
//
//void motorA_Backward() {
//  analogWrite(R_PWM_A, PWM);
//  analogWrite(L_PWM_A, 0);
//}
//
//void motorA_Forward_Slow() {
//  analogWrite(R_PWM_A, 0);
//  analogWrite(L_PWM_A, PWMSLOW);
//}
//
//void motorA_Backward_Slow() {
//  analogWrite(R_PWM_A, PWMSLOW);
//  analogWrite(L_PWM_A, 0);
//}
//
//void motorA_Stop() {
//  analogWrite(R_PWM_A, 0);
//  analogWrite(L_PWM_A, 0);
//}
//
////==================== motor B (left) control
//void motorB_Forward() {
//  analogWrite(R_PWM_B, 0);
//  analogWrite(L_PWM_B, PWM);
//}
//
//void motorB_Backward() {
//  analogWrite(R_PWM_B, PWM);
//  analogWrite(L_PWM_B, 0);
//}
//
//void motorB_Forward_Slow() {
//  analogWrite(R_PWM_B, 0);
//  analogWrite(L_PWM_B, PWMSLOW);
//}
//
//void motorB_Backward_Slow() {
//  analogWrite(R_PWM_B, PWMSLOW);
//  analogWrite(L_PWM_B, 0);
//}
//
//void motorB_Stop() {
//  analogWrite(R_PWM_B, 0);
//  analogWrite(L_PWM_B, 0);
//}
//
////รับค่าลองจิจูด
////double getLongitude(){
////    double robot_car_lon;
////    while(Serial2.available() > 0){
////      gps.encode(Serial2.read());
////      {
////        if(gps.location.isUpdated()){
////          robot_car_lon = gps.location.lng();
////        }
////      }
////    }
////    return robot_car_lon;
////}
//
////รับค่าละติจูด
////double getLatitude(){
////    double robot_car_lat;
////    while(Serial2.available() > 0){
////      gps.encode(Serial2.read());
////      {
////        if(gps.location.isUpdated()){
////          robot_car_lat = gps.location.lat();
////        }
////      }
////    }
////    return robot_car_lat;
////}
//
////เปลี่ยนค่าจากเรียนเดียนเป็นองศา
//double toDegree(double radian){
//    radian = (radian * 180) / PI;
//    return radian;
//}
//
//double toRadians( double degree){
//    double one_deg = PI / 180;
//    return (one_deg * degree);
//}
//
////มุมที่รถหันหน้าไป เทียบกับทิศเหนือ(azimuth ของรถ)
//int getCompassHeading(){
//    int x, y, z;
//    int robot_car_heading;
//  
//    qmc.read(&x, &y, &z, &robot_car_heading);
//  
//    robot_car_heading = qmc.azimuth(&y,&x);
//
////    Serial.print("Heading : ");
////    Serial.print(robot_car_heading);
//  
//    return robot_car_heading;  
//}
//
////คำนวณระยะทางระหว่าง 2 จุด
//double computeDistance(double waypoint_lat, double waypoint_lon, double robot_car_lat, double robot_car_lon) {
//    double lat2 = toRadians(waypoint_lat);
//    double lon2 = toRadians(waypoint_lon);
//    double lat1 = toRadians(robot_car_lat);
//    double lon1 = toRadians(robot_car_lon);
//
//    double dlat = lat2 - lat1;
//    double dlon = lon2 - lon1;
//    double a = pow(sin(dlat/2),2) + cos(lat1) * cos(lat2) * pow(sin(dlon/2),2);
//
//    double c = 2 * atan2(sqrt(a), sqrt(1-a));
//
//    double world_rarius = 6371.0;
//
//    double d = c * world_rarius;
//
//    double ans = d * pow(10,3);
//
//    Serial.print("\tdistance : ");
//    Serial.print(ans);
//    Serial.print(" m");
//
//    return ans; //distance between two point (m)
//}
//
////=======================หามุม azimuth ของพิกัดปลายทาง ===========================
//double computeAngle(double lat1, double lon1, double lat2, double lon2, int robot_car_heading){
//    lat2 = toRadians(lat2);
//    lon2 = toRadians(lon2);
//    lat1 = toRadians(lat1);
//    lon1 = toRadians(lon1);
//    double dlat = lat2 - lat1;
//    double dlon = lon2 - lon1;
//
//    double azimuth_angle = atan2( sin(dlon)*cos(lat2), (cos(lat1) * sin(lat2)) - (sin(lat1) * cos(lat2) * cos(dlon)));
//    int waypoint_angle = toDegree(azimuth_angle);
//
//    Serial.print(" azimuth : ");
//    waypoint_angle = 360 + waypoint_angle;
//    Serial.print(waypoint_angle);
//    int heading_error = waypoint_angle - robot_car_heading;
//    
//    if(heading_error > 180){
//      heading_error -= 360;
////      Serial.print("\theading error : ");
////      Serial.print(heading_error);
//      return heading_error;
//    }
////    Serial.print("\theading error : ");
////    Serial.print(heading_error);  
//    return(heading_error);
//}
//
//void setup() {
//  //=====motor driver pin =====
//  pinMode(R_PWM_A, OUTPUT);
//  pinMode(L_PWM_A, OUTPUT);
//  pinMode(R_PWM_B, OUTPUT);
//  pinMode(L_PWM_B, OUTPUT);
//  pinMode(R_EN_A, INPUT);
//  pinMode(L_EN_A, INPUT);
//  pinMode(R_EN_B, INPUT);
//  pinMode(L_EN_B, INPUT);
//  digitalWrite(R_EN_A, HIGH);
//  digitalWrite(L_EN_A, HIGH);
//  digitalWrite(R_EN_B, HIGH);
//  digitalWrite(L_EN_B, HIGH);
//  
//  Wire.begin();
//  Serial.begin(9600);
//  //Serial2.begin(GPSBaud);
//  qmc.init();
//}
//
//void loop() {  
//  struct robot_car_coordinates robot_car;
//  struct t_waypoint waypoint;
//
////=======================test functions==========================
//  
////  waypoint.latitude = 7.007539;
////  waypoint.longitude = 100.502696;  
////  robot_car.latitude = 7.007132;
////  robot_car.longitude = 100.502289;
//
////  waypoint.latitude = 7.007255;
////  waypoint.longitude = 100.502468;  
////  robot_car.latitude = 7.007132;
////  robot_car.longitude = 100.502289;
//
//  waypoint.latitude = 7.006962;
//  waypoint.longitude = 100.501513;  
//  robot_car.latitude = 7.007132;
//  robot_car.longitude = 100.502185;
//  
//  double distance = computeDistance(waypoint.latitude, waypoint.longitude, robot_car.latitude, robot_car.longitude);
//  int robot_car_heading = getCompassHeading();
//  double heading_error = computeAngle(robot_car.latitude, robot_car.longitude, waypoint.latitude, waypoint.longitude, robot_car_heading);  
//
////  while(computeDistance(waypoint.latitude, waypoint.longitude, robot_car.latitude, robot_car.longitude) > 3){
////
////    while(computeAngle(robot_car.latitude, robot_car.longitude, waypoint.latitude, waypoint.longitude, getCompassHeading()) < 5 || computeAngle(robot_car.latitude, robot_car.longitude, waypoint.latitude, waypoint.longitude, getCompassHeading()) > 5){
////       robot_car_heading = getCompassHeading();
////       Serial.print("Heading : ");
////       Serial.print(robot_car_heading);
////       heading_error = computeAngle(robot_car.latitude, robot_car.longitude, waypoint.latitude, waypoint.longitude, robot_car_heading);
////       Serial.print("\theading error : ");
////       Serial.print(heading_error);
////       
////      if(heading_error < -45 && heading_error >= -180){
////        turnLeft();
////      }
////      else if(heading_error < -5 && heading_error >= -45){
////        turnLeft_Slow();
////      }
////      else if(heading_error > 5 && heading_error <= 45){
////        turnRight_Slow();
////      }
////      else if(heading_error > 45 && heading_error <= 180){
////        turnRight();
////      }
////    }
////  }
////===================================
////       if(distance > 3){
////        robot_car_heading = getCompassHeading();
////        Serial.print("\tHeading : ");
////        Serial.print(robot_car_heading);
////        heading_error = computeAngle(robot_car.latitude, robot_car.longitude, waypoint.latitude, waypoint.longitude, robot_car_heading);
////        Serial.print("\theading error : ");
////        Serial.print(heading_error);
////         
////        if(heading_error < -45 && heading_error >= -180){
////          turnLeft();
////        }
////        else if(heading_error < -5 && heading_error >= -45){
////          turnLeft_Slow();
////        }
////        else if(heading_error > 5 && heading_error <= 45){
////          turnRight_Slow();
////        }
////        else if(heading_error > 45 && heading_error <= 180){
////          turnRight();
////        }
////        else {
////          forward();
////        }
////      }
////      else {
////        brake(); 
////      }
//      
//
////===============================================================
//
//    while (Serial2.available() > 0){
//      //data from the GPS
//      gps.encode(Serial2.read());
//        if (gps.location.isUpdated()){
//            robot_car.latitude = gps.location.lat();
//            robot_car.longitude = gps.location.lng();
//            Serial.print("Latitude= "); 
//            Serial.print(robot_car.latitude, 6);
//            Serial.print(" Longitude= "); 
//            Serial.print(robot_car.longitude, 6);
//            
//            int robot_car_heading = getCompassHeading();
//            
//            waypoint.latitude = 7.008651;
//            waypoint.longitude = 100.501968;
//            
//            double distance = computeDistance(waypoint.latitude, waypoint.longitude, robot_car.latitude, robot_car.longitude);
//            double heading_error = computeAngle(robot_car.latitude, robot_car.longitude, waypoint.latitude, waypoint.longitude, robot_car_heading);  
//            if(distance > 3){
//              if(){
//                
//              }
//                robot_car_heading = getCompassHeading();
//                Serial.print("\tHeading : ");
//                Serial.print(robot_car_heading);
//                heading_error = computeAngle(robot_car.latitude, robot_car.longitude, waypoint.latitude, waypoint.longitude, robot_car_heading);
//                Serial.print("\theading error : ");
//                Serial.print(heading_error);
//                 
//                if(heading_error < -45 && heading_error >= -180){
//                  turnLeft();
//                }
//                else if(heading_error < -5 && heading_error >= -45){
//                  turnLeft_Slow();
//                }
//                else if(heading_error > 5 && heading_error <= 45){
//                  turnRight_Slow();
//                }
//                else if(heading_error > 45 && heading_error <= 180){
//                  turnRight();
//                }
//                else {
//                  forward();
//                }
//            }
//            else {
//              brake(); 
//            }  
//        }
//    }
//}

//==========================================================================================================================================
//==========================================================================================================================================
#include<math.h>
#include <Wire.h>
#include <MechaQMC5883.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//#define pi 3.14159265358979323846
//==================== motor A (right)
#define R_PWM_A 4      //IN1
#define L_PWM_A 5      //IN2
#define R_EN_A 22
#define L_EN_A 23
//==================== motor B (left)
#define R_PWM_B 6      //IN3
#define L_PWM_B 7      //IN4
#define R_EN_B 24
#define L_EN_B 25

int PWM = 120;
int PWMSLOW = 80;
int maxSensor = 30;

//กำหนดพินของ ultrasonic secsor
int echoPin1 = 30;
int trigPin1 = 31;
int echoPin2 = 32;
int trigPin2 = 33;
int echoPin3 = 34;
int trigPin3 = 35;
int echoPin4 = 36;
int trigPin4 = 37;
// ตัวแปรสำหรับใช้ในฟังก์ชันการทำงานของ ultrasonic
long uDuration; 
int uDistance;

static const uint32_t GPSBaud = 9600;

//gps module
TinyGPSPlus gps;
//digital compass
MechaQMC5883 qmc;

struct t_waypoint{
  float latitude;
  float longitude;
};

struct robot_car_coordinates{
  float latitude;
  float longitude;
};

//============ navigation control ==============
void forward() {
  Serial.println("\tForward");
  motorA_Forward();
  motorB_Forward();
}

void backward() {
  Serial.println("\tBackward");
  motorA_Backward();
  motorB_Backward();
}

void turnRight() {
  Serial.println("\tTurn Right");
  motorA_Stop();
  motorB_Forward();
}

void turnLeft() {
  Serial.println("\tTurn Left");
  motorA_Forward();
  motorB_Stop();
}

void turnRight_Slow() {
  Serial.println("\tTurn Right Slow");
  motorA_Backward_Slow();
  motorB_Forward_Slow();
}

void turnLeft_Slow() {
  Serial.println("\tTurn Left Slow");
  motorA_Forward_Slow();
  motorB_Backward_Slow();
}

void brake() {
  Serial.println("\tBrake");
  motorA_Stop();
  motorB_Stop();
}

//void stop_motor() {
//  Serial.print("Motor stop");
//  motorA_Stop();
//  motorB_Stop();
//}

//==================== motor A (right) control
void motorA_Forward() {
  analogWrite(R_PWM_A, 0);
  analogWrite(L_PWM_A, PWM);
}

void motorA_Backward() {
  analogWrite(R_PWM_A, PWM);
  analogWrite(L_PWM_A, 0);
}

void motorA_Forward_Slow() {
  analogWrite(R_PWM_A, 0);
  analogWrite(L_PWM_A, PWMSLOW);
}

void motorA_Backward_Slow() {
  analogWrite(R_PWM_A, PWMSLOW);
  analogWrite(L_PWM_A, 0);
}

void motorA_Stop() {
  analogWrite(R_PWM_A, 0);
  analogWrite(L_PWM_A, 0);
}

//==================== motor B (left) control
void motorB_Forward() {
  analogWrite(R_PWM_B, 0);
  analogWrite(L_PWM_B, PWM);
}

void motorB_Backward() {
  analogWrite(R_PWM_B, PWM);
  analogWrite(L_PWM_B, 0);
}

void motorB_Forward_Slow() {
  analogWrite(R_PWM_B, 0);
  analogWrite(L_PWM_B, PWMSLOW);
}

void motorB_Backward_Slow() {
  analogWrite(R_PWM_B, PWMSLOW);
  analogWrite(L_PWM_B, 0);
}

void motorB_Stop() {
  analogWrite(R_PWM_B, 0);
  analogWrite(L_PWM_B, 0);
}

//รับค่าลองจิจูด
//double getLongitude(){
//    double robot_car_lon;
//    while(Serial2.available() > 0){
//      gps.encode(Serial2.read());
//      {
//        if(gps.location.isUpdated()){
//          robot_car_lon = gps.location.lng();
//        }
//      }
//    }
//    return robot_car_lon;
//}

//รับค่าละติจูด
//double getLatitude(){
//    double robot_car_lat;
//    while(Serial2.available() > 0){
//      gps.encode(Serial2.read());
//      {
//        if(gps.location.isUpdated()){
//          robot_car_lat = gps.location.lat();
//        }
//      }
//    }
//    return robot_car_lat;
//}

//เปลี่ยนค่าจากเรียนเดียนเป็นองศา
double toDegree(double radian){
    radian = (radian * 180) / PI;
    return radian;
}

double toRadians( double degree){
    double one_deg = PI / 180;
    return (one_deg * degree);
}

//มุมที่รถหันหน้าไป เทียบกับทิศเหนือ(azimuth ของรถ)
int getCompassHeading(){
    int x, y, z;
    int robot_car_heading;
  
    qmc.read(&x, &y, &z, &robot_car_heading);
  
    robot_car_heading = qmc.azimuth(&y,&x);

//    Serial.print("Heading : ");
//    Serial.print(robot_car_heading);
  
    return robot_car_heading;  
}

//คำนวณระยะทางระหว่าง 2 จุด
double computeDistance(double waypoint_lat, double waypoint_lon, double robot_car_lat, double robot_car_lon) {
    double lat2 = toRadians(waypoint_lat);
    double lon2 = toRadians(waypoint_lon);
    double lat1 = toRadians(robot_car_lat);
    double lon1 = toRadians(robot_car_lon);

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double a = pow(sin(dlat/2),2) + cos(lat1) * cos(lat2) * pow(sin(dlon/2),2);

    double c = 2 * atan2(sqrt(a), sqrt(1-a));

    double world_rarius = 6371.0;

    double d = c * world_rarius;

    double ans = d * pow(10,3);

    Serial.print("distance : ");
    Serial.print(ans);
    Serial.println(" m ");

    return ans; //distance between two point (m)
}

//=======================หามุม azimuth ของพิกัดปลายทาง ===========================
double computeAngle(double lat1, double lon1, double lat2, double lon2, int robot_car_heading){
    lat2 = toRadians(lat2);
    lon2 = toRadians(lon2);
    lat1 = toRadians(lat1);
    lon1 = toRadians(lon1);
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double azimuth_angle = atan2( sin(dlon)*cos(lat2), (cos(lat1) * sin(lat2)) - (sin(lat1) * cos(lat2) * cos(dlon)));
    int waypoint_angle = toDegree(azimuth_angle);

//    Serial.print(" azimuth : ");
    waypoint_angle = 360 + waypoint_angle;
//    Serial.print(waypoint_angle);
    int heading_error = waypoint_angle - robot_car_heading;
    
    if(heading_error > 180){
      heading_error -= 360;
      return heading_error;
    }
  
    return(heading_error);
}

long getLength(int echoPin, int trigPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  uDuration = pulseIn(echoPin, HIGH);
  uDistance = uDuration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  return uDistance;
}

void setup() {
  //===== motor driver pin =====
  pinMode(R_PWM_A, OUTPUT);
  pinMode(L_PWM_A, OUTPUT);
  pinMode(R_PWM_B, OUTPUT);
  pinMode(L_PWM_B, OUTPUT);
  pinMode(R_EN_A, INPUT);
  pinMode(L_EN_A, INPUT);
  pinMode(R_EN_B, INPUT);
  pinMode(L_EN_B, INPUT);
  digitalWrite(R_EN_A, HIGH);
  digitalWrite(L_EN_A, HIGH);
  digitalWrite(R_EN_B, HIGH);
  digitalWrite(L_EN_B, HIGH);

  //===== ultrasonic pin =====
  pinMode(trigPin1, OUTPUT); 
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT); 
  pinMode(echoPin2, INPUT); 
  pinMode(trigPin3, OUTPUT); 
  pinMode(echoPin3, INPUT);  
  pinMode(trigPin4, OUTPUT); 
  pinMode(echoPin4, INPUT);
  
  Wire.begin();
  Serial.begin(9600);
  Serial3.begin(115200);
  Serial2.begin(GPSBaud);
  qmc.init();
}

void loop() {  
  struct robot_car_coordinates robot_car;
  struct t_waypoint waypoint;
  
//  waypoint.latitude = 7.006905;
//  waypoint.longitude = 100.502260;  
//  robot_car.latitude = 7.007057;
//  robot_car.longitude = 100.502220;

    while ( Serial3.available() > 0 && Serial2.available() > 0){
    
      //data from the GPS
      gps.encode(Serial2.read());
        if (gps.location.isUpdated()){
            float val1 = Serial3.parseFloat();
            float val2 = Serial3.parseFloat();

//            Serial.print(val1);
//            Serial.print(" : ");
//            Serial.print(val2);
//            Serial.print("\t");
            
            float receivedLat;
            float receivedLon;

            if(val1>=1000000 && val1<2000000 && val2 >= 2000000 && val2<3000000){
              receivedLat = val1 - 1000000;
              waypoint.latitude = 7 + (receivedLat/1000000);
              
              receivedLon = val2 - 2000000;
              waypoint.longitude = 100 + (receivedLon/1000000) ;
              
            }  

            if(val2>=1000000 && val2<2000000 && val1 >= 2000000 && val1<3000000){
              receivedLat = val1 - 2000000;
              waypoint.latitude = 100 + (receivedLat/1000000);
              
              receivedLon = val2 - 1000000;
              waypoint.longitude = 7 + (receivedLon/1000000) ;

              float str_coo;
              str_coo = waypoint.latitude;
              waypoint.latitude = waypoint.longitude;
              waypoint.longitude = str_coo;
              
            } 
            
            robot_car.latitude = gps.location.lat();
            robot_car.longitude = gps.location.lng();
            
            Serial.print("lat: ");
            Serial.print(waypoint.latitude, 6);
            Serial.print(" ");
            Serial.print("long: ");
            Serial.print(waypoint.longitude, 6);
            Serial.print("\t");
            Serial.print("RobotCarLatitude= "); 
            Serial.print(robot_car.latitude, 6);
            Serial.print(" RobotCarLongitude= "); 
            Serial.print(robot_car.longitude, 6);
            Serial.print(" ");
            
            int robot_car_heading = getCompassHeading();
            Serial.print("heading: ");
            Serial.print(robot_car_heading);
            Serial.print(" ");

            double distance = computeDistance(waypoint.latitude, waypoint.longitude, robot_car.latitude, robot_car.longitude);
            double heading_error = computeAngle(robot_car.latitude, robot_car.longitude, waypoint.latitude, waypoint.longitude, robot_car_heading);  
              if(distance > 3){
                  robot_car_heading = getCompassHeading();
                  Serial.print("\tHeading : ");
                  Serial.print(robot_car_heading);
                  heading_error = computeAngle(robot_car.latitude, robot_car.longitude, waypoint.latitude, waypoint.longitude, robot_car_heading);
                  Serial.print("\theading error : ");
                  Serial.println(heading_error);
                   
                  if(heading_error < -45 && heading_error >= -180){
                    turnLeft();
                  }
                  else if(heading_error < -5 && heading_error >= -45){
                    turnLeft_Slow();
                  }
                  else if(heading_error > 5 && heading_error <= 45){
                    turnRight_Slow();
                  }
                  else if(heading_error > 45 && heading_error <= 180){
                    turnRight();
                  }
                  else {
                    forward();
                  }
//                }
                //ระยะทางน้อยกว่า 3m (ถึงจุดหมาย)
                
              }
              else {
                  brake(); 
              }                   
         }// gps
        
    }

}

//==========================================================================================================================================
//==========================================================================================================================================
