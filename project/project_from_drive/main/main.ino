#include<math.h>
#include <Wire.h>
#include <MechaQMC5883.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define pi 3.14159265358979323846
#define in1PinRA 6
#define in2PinRA 7
#define in3PinLB 4
#define in4PinLB 5
#define enableRight 10
#define enableLeft 11 

//----------------------ultrasonic sensor----------------------------

int echoPin1 = 30;
int trigPin1 = 31;
int echoPin2 = 32;
int trigPin2 = 33;
int echoPin3 = 34;
int trigPin3 = 35;
long u_duration; 
int u_distance;

int is_turnleft = 0;
static const int GPSRXPin = 4;
static const int GPSTXPin = 3;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
MechaQMC5883 qmc;

SoftwareSerial ssgps(GPSRXPin, GPSTXPin);
SoftwareSerial ArduinoSerialTransmitter(18, 19);

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
          lat2 = gps.location.lat();
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
    double angle_ref_x;

    angle_ref_x = atan(dlat / dlon); //result is radians
    angle_ref_x = to_degree(angle_ref_x); //result is degree

//    printf("%f\n", angle_ref_x);
    
    is_turnleft = 0;//-----------------------------------------------------------------------------turn right
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

void enableMotor(){
  motorRightOn();
  motorLeftOn();
}

void disableMotor(){
  motorRightOff();
  motorLeftOff();
}

void forward(int time_val){
  motorRightForward();
  motorLeftForward();
  delay(time_val);
}

void backward(int time_val){
  motorRightBackward();
  motorLeftBackward();
  delay(time_val);
}

void turnRight(int time_val){
  motorRightBrake();
  motorLeftForward();
  delay(time_val);
}

void turnLeft(int time_val){
  motorRightForward();
  motorLeftBrake();
  delay(time_val);
}

void uturn(int time_val){
  motorRightForward();
  motorLeftBackward();
  delay(time_val);
}

void coast(int time_val){
  motorRightCoast();
  motorLeftCoast();
  delay(time_val);
}

void brake(){
  motorRightBrake();
  motorLeftBrake();
}

//enable motor
void motorRightOn(){
  digitalWrite(enableRight, HIGH);
}

void motorLeftOn(){
  digitalWrite(enableLeft, HIGH);
}

//disable motor
void motorRightOff(){
  digitalWrite(enableRight, LOW);
}

void motorLeftOff(){
  digitalWrite(enableLeft, LOW);
}

// -------------------------------------------motor Right control
void motorRightForward(){
  digitalWrite(in1PinRA, HIGH);
  digitalWrite(in2PinRA, LOW);
}

void motorRightBackward(){
  digitalWrite(in1PinRA, LOW);
  digitalWrite(in2PinRA, HIGH);
}

// -----------------------------------------motor Left control
void motorLeftForward(){
  digitalWrite(in3PinLB, HIGH);
  digitalWrite(in4PinLB, LOW);
}

void motorLeftBackward(){
  digitalWrite(in3PinLB, LOW);
  digitalWrite(in4PinLB, HIGH);
}

void motorRightBrake(){
  digitalWrite(in1PinRA, LOW);
  digitalWrite(in2PinRA, LOW);
}

void motorLeftBrake(){
  digitalWrite(in3PinLB, LOW);
  digitalWrite(in4PinLB, LOW);
}

void motorRightCoast(){
  digitalWrite(in1PinRA, LOW);
  digitalWrite(in2PinRA, LOW);
}

void motorLeftCoast(){
  digitalWrite(in3PinLB, LOW);
  digitalWrite(in4PinLB, LOW);
}

int azimuth_angle(){
    int x, y, z;
    int azimuth_ang;
  
    qmc.read(&x, &y, &z, &azimuth_ang);
  
    azimuth_ang = qmc.azimuth(&y,&x);
  
//    Serial.println();
//    Serial.print("x: ");
//    Serial.print(x);
//    Serial.print(" y: ");
//    Serial.print(y);
//    Serial.print(" z: ");
//    Serial.print(z);
//    Serial.print(" azimuth: ");
//    Serial.print(azimuth_ang);
//    Serial.print("     ");
  
    return azimuth_ang;
    
}

long getLength(int echoPin, int trigPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  u_duration = pulseIn(echoPin, HIGH);
  u_distance = u_duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  return u_distance;
}

void printUDistance(int u1, int u2, int u3){
    Serial.print(u1);
    Serial.print(" : ");
    Serial.print(u2);
    Serial.print(" : ");
    Serial.print(u3);
    Serial.print(" cm ");
}

void setup() {
    //------------ultrasonic sensor----------------------
    pinMode(trigPin1, OUTPUT); 
    pinMode(echoPin1, INPUT);
    pinMode(trigPin2, OUTPUT); 
    pinMode(echoPin2, INPUT); 
    pinMode(trigPin3, OUTPUT); 
    pinMode(echoPin3, INPUT);
    //-----------------motor-----------------------------
    pinMode(in1PinRA, OUTPUT);
    pinMode(in2PinRA, OUTPUT);
    pinMode(in3PinLB, OUTPUT);
    pinMode(in4PinLB, OUTPUT);
    pinMode(enableRight, OUTPUT);
    pinMode(enableLeft, OUTPUT);
    Wire.begin();
    Serial.begin(9600);
    ArduinoSerialTransmitter.begin(57600);
    ssgps.begin(GPSBaud);
    qmc.init();

}

void loop() {
    float lat1 = 18.741318;          //y1
    float lon1 = 98.943172;          //x1
//    double lat2 = getLattitude();     //y2
//    double lon2 = getLongitude();     //x2
    float lat2 = 18.741928;          //y2
    float lon2 = 98.941476;          //x2

    analogWrite(enableLeft, 200);
    analogWrite(enableRight, 200);
    
    azimuth_angle();
    double car_angle_ref_n = azimuth_angle();
    double rotate_ang = rotate_angle(lat1, lon1, lat2, lon2, car_angle_ref_n);

    while(distance > 3){       // เช็คว่าตำแหน่งตรงกันหรือไม่ จากระยะห่าง ค่าความผิดพลาด 3m
          long u1 = getLength(echoPin1, trigPin1); // วัดระยะทาง Ultrasonic ตัวที่ 1
          long u2 = getLength(echoPin2, trigPin2); // วัดระยะทาง Ultrasonic ตัวที่ 2
          long u3 = getLength(echoPin3, trigPin3); // วัดระยะทาง Ultrasonic ตัวที่ 3
          printUDistance(u1, u2, u3);
         
//       while(rotate_ang > 2){
//         if(rotate_ang <= 180){
//           turnRight(1);
//         }
//         else{
//           turnLeft(1);
//         }
//       }
       
       if(u1 <= 20){ //--------------เจอสิ่งกีดขวางทางซ้าย
            Serial.println("Turn Right");
          coast(2000);
          backward(1000);
          coast(2000);
          turnRight(1000);
          coast(2000);
       }
       else if(u2 <= 20){ //--------------เจอสิ่งกีดขวางตรงกลาง
            Serial.println("Turn Right");
          coast(2000);
          backward(1000);
          coast(2000);
          turnRight(1000);
          coast(2000);
       }
       else if(u3 <= 20){ //--------------เจอสิ่งกีดขวางทางขวา
            Serial.println("Turn Left");
          coast(2000);
          backward(1000);
          coast(2000);
          turnLeft(1000);
          coast(2000);
       }
       else{  //--------------ไม่เจอเจอสิ่งกีดขวางในระยะ
            Serial.println("Forward");
          forward(1);
       }
    }
    brake();
    ArduinoSerialTransmitter.print(lat1);
    ArduinoSerialTransmitter.print(" ");
    ArduinoSerialTransmitter.print(lon1);  
    ArduinoSerialTransmitter.print(" ");
    ArduinoSerialTransmitter.print(lat2);
    ArduinoSerialTransmitter.print(" ");
    ArduinoSerialTransmitter.print(lon2);  
    ArduinoSerialTransmitter.print("\n");
    delay(1000);

//    Serial.print("Lat1= "); 
//    Serial.print(lat1);
//    Serial.print(" Lon1= "); 
//    Serial.print(lon1); 
//    Serial.print("Lat2= "); 
//    Serial.print(lat2);
//    Serial.print(" Lon2= "); 
//    Serial.print(lon2);      
//    Serial.print(" rotate angle : ");
//    Serial.print(rotate_ang);
//    Serial.print("\t distance : ");
//    Serial.print(distance(lat1, lon1, lat2, lon2));
//    Serial.println(" m");

}
