#include <TimerOne.h>
#include<math.h>
#include <Wire.h>
#include <MechaQMC5883.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//==================== motor A (right)
#define R_PWM_A 8      //IN1
#define L_PWM_A 9      //IN2
#define R_EN_A 22
#define L_EN_A 23
//==================== motor B (left)
#define R_PWM_B 6      //IN3
#define L_PWM_B 7      //IN4
#define R_EN_B 24
#define L_EN_B 25

#define trigPinC 33                                    // Pin 33 trigger output
#define echoPinC 2                                     // Pin 2 Echo input
#define echo_intC 0                                    // Interrupt id for echo pulse

#define trigPinR 35                                    // Pin 35 trigger output
#define echoPinR 3                                     // Pin 3 Echo input
#define echo_intR 1

#define trigPinL 31                                     // Pin 31 trigger output
#define echoPinL 18                                     // Pin 18 Echo input
#define echo_intL 5

#define TIMER_US 50                                   // 50 uS timer duration 
#define TICK_COUNTS 4000                              // 200 mS worth of timer ticks

volatile long echo_startC = 0;                         // Records start of echo pulse 
volatile long echo_endC = 0;                           // Records end of echo pulse
volatile long echo_durationC = 0;                      // Duration - difference between end and start
volatile long echo_startL = 0;                         // Records start of echo pulse 
volatile long echo_endL = 0;                           // Records end of echo pulse
volatile long echo_durationL = 0;                      // Duration - difference between end and start
volatile long echo_startR = 0;                         // Records start of echo pulse 
volatile long echo_endR = 0;                           // Records end of echo pulse
volatile long echo_durationR = 0;                      // Duration - difference between end and start
volatile int trigger_time_countC = 0;                  // Count down counter to trigger pulse time
volatile int trigger_time_countL = 0;
volatile int trigger_time_countR = 0;

int PWMFAST = 180;                                      //PWM ของ motor
int PWM = 100;
int PWMSLOW = 80;
int max_sensor = 50;                                    // ระยะที่ตรวจจับได้ว่ามีสิ่งกีดขวาง
int avoiding_backward = 1000;                           // ระยะเวลาในการทำงานให้หลบสิ่งกีดขวาง
int avoiding_forward = 2000;                            // ระยะเวลาในการทำงานให้หลบสิ่งกีดขวาง
int avoiding_turn = 1500;                               // ระยะเวลาในการทำงานให้หลบสิ่งกีดขวาง
int avoiding_brake = 2000;                              // ระยะเวลาในการทำงานให้หลบสิ่งกีดขวาง

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

//============ navigation control ===========================
void forward(int val) {
  Serial.println("\tForward");
  motorA_Forward(val);
  motorB_Forward(val);
}

void backward(int val) {
  Serial.println("\tBackward");
  motorA_Backward(val);
  motorB_Backward(val);
}

void turnRight(int val) {
  Serial.println("\tTurn Right");
  motorA_Backward(val);
  motorB_Forward(val);
}

void turnLeft(int val) {
  Serial.println("\tTurn Left");
  motorA_Forward(val);
  motorB_Backward(val);
}

void turnRight_Slow(int val) {
  Serial.println("\tTurn Right Slow");
  motorA_Backward_Slow(val);
  motorB_Forward_Slow(val);
}

void turnLeft_Slow(int val) {
  Serial.println("\tTurn Left Slow");
  motorA_Forward_Slow(val);
  motorB_Backward_Slow(val);
}

void brake() {
  Serial.println("\tBrake");
  motorA_Stop();
  motorB_Stop();
}

//==================== motor A (right) control====================
void motorA_Forward_Fast(int val) {
  analogWrite(R_PWM_A, 0);
  analogWrite(L_PWM_A, val);
}

void motorA_Forward(int val) {
  analogWrite(R_PWM_A, 0);
  analogWrite(L_PWM_A, val);
}

void motorA_Backward(int val) {
  analogWrite(R_PWM_A, val);
  analogWrite(L_PWM_A, 0);
}

void motorA_Forward_Slow(int val) {
  analogWrite(R_PWM_A, 0);
  analogWrite(L_PWM_A, val);
}

void motorA_Backward_Slow(int val) {
  analogWrite(R_PWM_A, val);
  analogWrite(L_PWM_A, 0);
}

void motorA_Stop() {
  analogWrite(R_PWM_A, 0);
  analogWrite(L_PWM_A, 0);
}

//==================== motor B (left) control========================
void motorB_Forward_Fast(int val) {
  analogWrite(R_PWM_B, 0);
  analogWrite(L_PWM_B, val);
}

void motorB_Forward(int val) {
  analogWrite(R_PWM_B, 0);
  analogWrite(L_PWM_B, val);
}

void motorB_Backward(int val) {
  analogWrite(R_PWM_B, val);
  analogWrite(L_PWM_B, 0);
}

void motorB_Forward_Slow(int val) {
  analogWrite(R_PWM_B, 0);
  analogWrite(L_PWM_B, val);
}

void motorB_Backward_Slow(int val) {
  analogWrite(R_PWM_B, val);
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

//==========================มุมของรถเทียบกับทิศเหนือ(azimuth ของรถ)============================
int getCompassHeading(){
    int x, y, z;
    int robot_car_heading;
  
    qmc.read(&x, &y, &z, &robot_car_heading);
  
    robot_car_heading = qmc.azimuth(&y,&x);

//    Serial.print("Heading : ");
//    Serial.print(robot_car_heading);
  
    return robot_car_heading;  
}

//====================================คำนวณระยะทางระหว่าง 2 จุด==================================
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

//======================= หามุมที่รถต้องหันไปยังพิกัดเป้าหมาย ===========================
double computeAngle(double lat1, double lon1, double lat2, double lon2, int robot_car_heading){
    lat2 = toRadians(lat2);
    lon2 = toRadians(lon2);
    lat1 = toRadians(lat1);
    lon1 = toRadians(lon1);
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double azimuth_angle = atan2( sin(dlon)*cos(lat2), (cos(lat1) * sin(lat2)) - (sin(lat1) * cos(lat2) * cos(dlon)));    // azimuth ของพิกัดเป้าหมาย
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
    trigger_pulseC();                                 // Schedule the trigger pulses
    trigger_pulseL();                                 // Schedule the trigger pulses
    trigger_pulseR();                                 // Schedule the trigger pulses
}

void trigger_pulseC()
{
      static volatile int state = 0;                 // State machine variable

      if (!(--trigger_time_countC))                   // Count to 200mS
      {                                              // Time out - Initiate trigger pulse
         trigger_time_countC = TICK_COUNTS;           // Reload
         state = 1;                                  // Changing to state 1 initiates a pulse
      }
    
      switch(state)                                  // State machine handles delivery of trigger pulse
      {
        case 0:                                      // Normal state does nothing
            break;
        
        case 1:                                      // Initiate pulse
           digitalWrite(trigPinC, HIGH);              // Set the trigger output high
           state = 2;                                // and set state to 2
           break;
        
        case 2:                                      // Complete the pulse
        default:      
           digitalWrite(trigPinC, LOW);               // Set the trigger output low
           state = 0;                                // and return state to normal 0
           break;
     }
}
void trigger_pulseL()
{
      static volatile int state = 0;                 // State machine variable

      if (!(--trigger_time_countL))                   // Count to 200mS
      {                                              // Time out - Initiate trigger pulse
         trigger_time_countL = TICK_COUNTS;           // Reload
         state = 1;                                  // Changing to state 1 initiates a pulse
      }
    
      switch(state)                                  // State machine handles delivery of trigger pulse
      {
        case 0:                                      // Normal state does nothing
            break;
        
        case 1:                                      // Initiate pulse
           digitalWrite(trigPinL, HIGH);              // Set the trigger output high
           state = 2;                                // and set state to 2
           break;
        
        case 2:                                      // Complete the pulse
        default:      
           digitalWrite(trigPinL, LOW);               // Set the trigger output low
           state = 0;                                // and return state to normal 0
           break;
     }
}
void trigger_pulseR()
{
      static volatile int state = 0;                 // State machine variable

      if (!(--trigger_time_countR))                   // Count to 200mS
      {                                              // Time out - Initiate trigger pulse
         trigger_time_countR = TICK_COUNTS;           // Reload
         state = 1;                                  // Changing to state 1 initiates a pulse
      }
    
      switch(state)                                  // State machine handles delivery of trigger pulse
      {
        case 0:                                      // Normal state does nothing
            break;
        
        case 1:                                      // Initiate pulse
           digitalWrite(trigPinR, HIGH);              // Set the trigger output high
           state = 2;                                // and set state to 2
           break;
        
        case 2:                                      // Complete the pulse
        default:      
           digitalWrite(trigPinR, LOW);               // Set the trigger output low
           state = 0;                                // and return state to normal 0
           break;
     }
}

void echo_interruptC()
{
  switch (digitalRead(echoPinC))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_endC = 0;                                 // Clear the end time
      echo_startC = micros();                        // Save the start time
      break;
      
    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_endC = micros();                          // Save the end time
      echo_durationC = echo_endC - echo_startC;        // Calculate the pulse duration
      Serial.print("C : ");
      Serial.println(echo_durationC / 58);               // Print the distance in centimeters
      break;
  }
}
void echo_interruptL()
{
  switch (digitalRead(echoPinL))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_endL = 0;                                 // Clear the end time
      echo_startL = micros();                        // Save the start time
      break;
      
    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_endL = micros();                          // Save the end time
      echo_durationL = echo_endL - echo_startL;        // Calculate the pulse duration
      Serial.print("L : ");
      Serial.println(echo_durationL / 58);               // Print the distance in centimeters
      break;
  }
}
void echo_interruptR()
{
  switch (digitalRead(echoPinR))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_endR = 0;                                 // Clear the end time
      echo_startR = micros();                        // Save the start time
      break;
      
    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_endR = micros();                          // Save the end time
      echo_durationR = echo_endR - echo_startR;        // Calculate the pulse duration
      Serial.print("R : ");
      Serial.println(echo_durationR / 58);               // Print the distance in centimeters
      break;
  }
}

//==============เลี้ยวหลบไปทางซ้าย=================
void avoidingL(){
  backward(PWM);
  delay(1000);
  brake();
  delay(1000);
  turnLeft(PWM);
  delay(1500);
  brake();
  delay(1000);
  forward(PWM);
  delay(2000);
  brake();
  delay(1000);
}

//==============เลี้ยวหลบไปทางขวา=================
void avoidingR(){
  backward(PWM);
  delay(1000);
  brake();
  delay(1000);
  turnRight(PWM);
  delay(1500);
  brake();
  delay(1000);
  forward(PWM);
  delay(2000);
  brake();
  delay(1000);
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

  pinMode(trigPinC, OUTPUT);                           // Trigger pin set to output
  pinMode(echoPinC, INPUT);                            // Echo pin set to input
  pinMode(trigPinR, OUTPUT);                           // Trigger pin set to output
  pinMode(echoPinR, INPUT);                            // Echo pin set to input
  pinMode(trigPinL, OUTPUT);                           // Trigger pin set to output
  pinMode(echoPinL, INPUT);                            // Echo pin set to input
  
  Timer1.initialize(TIMER_US);                                // Initialise timer 1
  Timer1.attachInterrupt( timerIsr );                         // Attach interrupt to the timer service routine 
  attachInterrupt(echo_intC, echo_interruptC, CHANGE);        // Attach interrupt to the center sensor echo input
  attachInterrupt(echo_intL, echo_interruptL, CHANGE);        // Attach interrupt to the left sensor echo input
  attachInterrupt(echo_intR, echo_interruptR, CHANGE);        // Attach interrupt to the right sensor echo input

  Wire.begin();
  Serial.begin(9600);
  Serial3.begin(115200);
  Serial2.begin(GPSBaud);
  qmc.init();
}

void loop() {  
  struct robot_car_coordinates robot_car;                   // ตัวแปรสำหรับเก็บค่าพิกัดปัจจุบันของหุ่นยนต์
  struct t_waypoint waypoint;                               // ตัวแปรสำหรับเก็บค่าพิกัดเป้าหมาย

//  พิกัดข้างล่างนี้ ใช้สำหรับการทดสอบ
//  waypoint.latitude = 7.006595;
//  waypoint.longitude = 100.502193;
//  robot_car.latitude = 7.007057;
//  robot_car.longitude = 100.502220;

// เช็คสิ่งกีดขวาง
  if((echo_durationL / 58) <= 50){
      avoidingR();
  }
  else if((echo_durationC / 58) <= 50 || (echo_durationR / 58) <= 50){
      avoidingL();
  }
  else{
      if ( Serial3.available() > 0 && Serial2.available() > 0 ){
    
      //data from the GPS
      gps.encode(Serial2.read());                                   //อ่านและแปลงค่าจาก gps
        if (gps.location.isUpdated()){                              // gps มีการอัพเดทค่าพิกัด
            float val1 = Serial3.parseFloat();                      // รับค่าจาก node mcu                   
            float val2 = Serial3.parseFloat();                      // รับค่าจาก node mcu
           
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
            
            robot_car.latitude = gps.location.lat();                // เก็บค่าละติจูด
            robot_car.longitude = gps.location.lng();               // เก็บค่าลองจิจูด
            
            
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
            
            int robot_car_heading = getCompassHeading();           // คำนวณทิศทางของหุ่นว่าตอนนี้หันไปทางไหน เป็นมุมเทียบกับทิศเหนือ
            Serial.print("heading: ");
            Serial.print(robot_car_heading);
            Serial.print(" ");

            // คำนวณระยะทางระหว่างหุ่นกับพิกัดเป้าหมาย
            double distance = computeDistance(waypoint.latitude, waypoint.longitude, robot_car.latitude, robot_car.longitude);
            
            // คำนวณมุมที่หุ่นทำกับพิกัดเป้าหมาย
            double heading_error = computeAngle(robot_car.latitude, robot_car.longitude, waypoint.latitude, waypoint.longitude, robot_car_heading); 
             
            if(distance > 3){
                  robot_car_heading = getCompassHeading();
                  Serial.print("\tHeading : ");
                  Serial.print(robot_car_heading);
                  heading_error = computeAngle(robot_car.latitude, robot_car.longitude, waypoint.latitude, waypoint.longitude, robot_car_heading);
                  Serial.print("\theading error : ");
                  Serial.println(heading_error);
                   
                  if(heading_error < -45 && heading_error >= -180){
                    turnLeft(PWM);
                  }
                  else if(heading_error < -10 && heading_error >= -45){
                    turnLeft_Slow(PWMSLOW);
                  }
                  else if(heading_error > 10 && heading_error <= 45){
                    turnRight_Slow(PWMSLOW);
                  }
                  else if(heading_error > 45 && heading_error <= 180){
                    turnRight(PWM);
                  }
                  else {
                    forward(PWM);
                  }
                                
            }
            else {
              brake(); 
            }  
        }//gps local update

    }
    
  } //ultrasonic
    
}
