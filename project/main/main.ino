#include <TimerOne.h>
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

#define trigPin 33                                    // Pin  trigger output
#define echoPin 2                                     // Pin  Echo input
#define echo_int 0                                    // Interrupt id for echo pulse

#define TIMER_US 50                                   // 50 uS timer duration 
#define TICK_COUNTS 4000                              // 200 mS worth of timer ticks

int AVOIDING_TURNRIGHT = 40000;
int AVOIDING_BRAKE = 80000;
int AVOIDING_FORWARD = 120000;

volatile long echo_start = 0;                         // Records start of echo pulse 
volatile long echo_end = 0;                           // Records end of echo pulse
volatile long echo_duration = 0;                      // Duration - difference between end and start
volatile int trigger_time_count = 0;                  // Count down counter to trigger pulse time

int PWMFAST = 180;
int PWM = 150;
int PWMSLOW = 90;
int max_sensor = 50;
int avoiding_backward = 1000;
int avoiding_forward = 2000;
int avoiding_turn = 1500;
int avoiding_brake = 2000;

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
  motorA_Backward();
  motorB_Forward();
}

void turnLeft() {
  Serial.println("\tTurn Left");
  motorA_Forward();
  motorB_Backward();
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
void motorA_Forward_Fast() {
  analogWrite(R_PWM_A, 0);
  analogWrite(L_PWM_A, PWMFAST);
}

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
void motorB_Forward_Fast() {
  analogWrite(R_PWM_B, 0);
  analogWrite(L_PWM_B, PWMFAST);
}

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

void timerIsr()
{
    trigger_pulse();                                 // Schedule the trigger pulses
}

void trigger_pulse()
{
      static volatile int state = 0;                 // State machine variable

      if (!(--trigger_time_count))                   // Count to 200mS
      {                                              // Time out - Initiate trigger pulse
         trigger_time_count = TICK_COUNTS;           // Reload
         state = 1;                                  // Changing to state 1 initiates a pulse
      }
    
      switch(state)                                  // State machine handles delivery of trigger pulse
      {
        case 0:                                      // Normal state does nothing
            break;
        
        case 1:                                      // Initiate pulse
           digitalWrite(trigPin, HIGH);              // Set the trigger output high
           state = 2;                                // and set state to 2
           break;
        
        case 2:                                      // Complete the pulse
        default:      
           digitalWrite(trigPin, LOW);               // Set the trigger output low
           state = 0;                                // and return state to normal 0
           break;
     }
}

void echo_interrupt()
{
  switch (digitalRead(echoPin))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_end = 0;                                 // Clear the end time
      echo_start = micros();                        // Save the start time
      break;
      
    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_end = micros();                          // Save the end time
      echo_duration = echo_end - echo_start;        // Calculate the pulse duration
      Serial.println(echo_duration / 58);               // Print the distance in centimeters
      break;
  }
}

void avoiding(){
  brake();
  delay(2000);
  turnRight();
  delay(2000);
  forward();
  delay(2000);
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

  pinMode(trigPin, OUTPUT);                           // Trigger pin set to output
  pinMode(echoPin, INPUT);                            // Echo pin set to input
  
  Timer1.initialize(TIMER_US);                        // Initialise timer 1
  Timer1.attachInterrupt( timerIsr );                 // Attach interrupt to the timer service routine 
  attachInterrupt(echo_int, echo_interrupt, CHANGE);  // Attach interrupt to the sensor echo input

  Wire.begin();
  Serial.begin(9600);
  Serial3.begin(115200);
  Serial2.begin(GPSBaud);
  qmc.init();
}

void loop() {  
  struct robot_car_coordinates robot_car;
  struct t_waypoint waypoint;
  
//  waypoint.latitude = 7.006595;
//  waypoint.longitude = 100.502193;
  robot_car.latitude = 7.007057;
  robot_car.longitude = 100.502220;
  
  if((echo_duration / 58) <= 50){
      avoiding();
  }
  else{
      if ( Serial3.available() > 0 ){
    
      //data from the GPS
//      gps.encode(Serial2.read());
//        if (gps.location.isUpdated()){
            float val1 = Serial3.parseFloat();
            float val2 = Serial3.parseFloat();

//            Serial.print(val1);
//            Serial.print(" : ");
//            Serial.print(val2);
//            Serial.print("\t");
            
            float receivedLat;
            float receivedLon;
            
            //เช็คความถูกต้องของค่าพิกัดที่รับมาจาก prefix (latitude+100000, longitude+2000000)
            if(val1>=1000000 && val1<2000000 && val2 >= 2000000 && val2<3000000){
              receivedLat = val1 - 1000000;
              waypoint.latitude = 7 + (receivedLat/1000000);
              
              receivedLon = val2 - 2000000;
              waypoint.longitude = 100 + (receivedLon/1000000) ;
              
            }  
            //กรณีที่ค่าพิกัดที่รับมาสลับกัน
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
            
//            robot_car.latitude = gps.location.lat();
//            robot_car.longitude = gps.location.lng();
            
            
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
                  else if(heading_error < -10 && heading_error >= -45){
                    turnLeft_Slow();
                  }
                  else if(heading_error > 10 && heading_error <= 45){
                    turnRight_Slow();
                  }
                  else if(heading_error > 45 && heading_error <= 180){
                    turnRight();
                  }
                  else {
                    forward();
                  }
                                
            }
            else {
              brake(); 
            }  
//        }

    }
    
  }
    
}
