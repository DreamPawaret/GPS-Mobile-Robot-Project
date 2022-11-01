#include <TimerOne.h>
#include <math.h>
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

#define trigPinC 33                                    // Pin 12 trigger output
#define echoPinC 2                                     // Pin 2 Echo input
#define echo_intC 0                                    // Interrupt id for echo pulse

#define trigPinR 35                 
#define echoPinR 3                                     
#define echo_intR 1

#define trigPinL 31                 
#define echoPinL 18                                     
#define echo_intL 5

#define TIMER_US 50                                   // 50 uS timer duration 
#define TICK_COUNTS 4000                              // 200 mS worth of timer ticks

//====================================================
#define INTERVAL_TIME 1000                            // 1000 ms โปรแกรมนำทางหุ่นยนต์ทุกๆ 1 วินาที
unsigned long time_to_cal_gps = 0;

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

int PWMFAST = 180;
int PWM = 100;
int PWMSLOW = 80;
int max_sensor = 50;
int avoiding_backward = 1000;
int avoiding_forward = 2000;
int avoiding_turn = 1500;
int avoiding_brake = 2000;

int Mode;
int control_value;

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

//==================== motor A (right) control
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

//==================== motor B (left) control
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

//มุมของรถเทียบกับทิศเหนือ(azimuth ของรถ)
int getCompassHeading(){
    int x, y, z;
    int robot_car_heading;
  
    qmc.read(&x, &y, &z, &robot_car_heading);
  
    robot_car_heading = qmc.azimuth(&y,&x);

//    Serial.print("Heading : ");
//    Serial.print(robot_car_heading);
  
    return robot_car_heading;  
}

void storeGpsReading(float latitude_read,float longitude_read){
  
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

void sendPacket(byte *packet, byte len) {
 for (byte i = 0; i < len; i++)
 {
 Serial2.write(packet[i]); // GPS is HardwareSerial
 }
}

void changeFrequency() {
    byte packet[] = {
        0xB5, // 
        0x62, // 
        0x06, // 
        0x08, // 
        0x06, // length
        0x00, // 
        0xC8, // measRate, hex 64 = dec 100 ms, C8 = 200 ms, 1FE = 500ms, 3E8 = 1000ms
        0x00, // 
        0x01, // navRate, always =1
        0x00, // 
        0x01, // timeRef, stick to GPS time (=1)
        0x00, // 
        0x7A, // CK_A
        0x12, // CK_B
    };
    sendPacket(packet, sizeof(packet));
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
  
  Timer1.initialize(TIMER_US);                        // Initialise timer 1
  Timer1.attachInterrupt( timerIsr );                 // Attach interrupt to the timer service routine 
//  attachInterrupt(echo_intC, echo_interruptC, CHANGE);  // Attach interrupt to the sensor echo input
//  attachInterrupt(echo_intL, echo_interruptL, CHANGE);  // Attach interrupt to the sensor echo input
//  attachInterrupt(echo_intR, echo_interruptR, CHANGE);  // Attach interrupt to the sensor echo input

  Wire.begin();
  Serial.begin(9600); 
  Serial3.begin(115200);
  Serial2.begin(GPSBaud);   //gps serial
  qmc.init();
//  changeFrequency();
//  delay(200); Serial2.flush();
}

void loop() {  
  struct robot_car_coordinates robot_car_read[5];
  struct robot_car_coordinates robot_car;
  struct t_waypoint waypoint;
  
//  waypoint.latitude = 7.006595;
//  waypoint.longitude = 100.502193;
  robot_car_read[0].latitude = 7.007057;
  robot_car_read[0].longitude = 100.502220;
  robot_car_read[1].latitude = 7.007058;
  robot_car_read[1].longitude = 100.502221;
  robot_car_read[2].latitude = 7.007059;
  robot_car_read[2].longitude = 100.502222;
  robot_car_read[3].latitude = 7.007055;
  robot_car_read[3].longitude = 100.502219;
  robot_car_read[4].latitude = 7.007056;
  robot_car_read[4].longitude = 100.502218;
//  robot_car.latitude = 0;
//  robot_car.longitude = 0;
  
      if ( Serial3.available() > 0 /*&& Serial2.available() > 0*/ ){
        
//        Mode = Serial3.parseInt();
//        control_value = Serial3.parseInt();
//        Serial.println(Mode);
//        if(Mode == 0){
//          if(control_value == 7){
//             forward(PWM);
//          }
//          else if(control_value == 4){
//            turnRight(PWM);
//          }
//          else if(control_value == 8){
//            backward(PWM);
//          }
//          else if(control_value == 3){
//            turnLeft(PWM);
//          }
//          else{
//            brake();
//          }
//        }
//        else{ //mode
//          if((echo_durationL / 58) <= 50){
//              avoidingR();
//          }
//          else if((echo_durationC / 58) <= 40 || (echo_durationR / 58) <= 50){
//              avoidingL();
//          }
//          else{ //echo_duration
          
            //data from the GPS
//            gps.encode(Serial2.read());
//            if (gps.location.isUpdated()){
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
                
//                robot_car.latitude = gps.location.lat();
//                robot_car.longitude = gps.location.lng();
                int i = 0;
                if(i < 5){
                  robot_car_read[i].latitude = gps.location.lat();
                  robot_car_read[i].longitude = gps.location.lng();
//                  robot_car_read[i].latitude = robot_car_read[i];
//                  robot_car_read[i].longitude = robot_car_read[i];
                    i++;
                }
                else {
                  i = i-5;
                }
                
                if(millis() - time_to_cal_gps > INTERVAL_TIME){
                  time_to_cal_gps = millis();
                  for(int i=0 ; i<5 ; i++){
                    robot_car.latitude += robot_car_read[i].latitude;
                    robot_car.longitude += robot_car_read[i].longitude;
                  }
                  robot_car.latitude = robot_car.latitude / 5;
                  robot_car.longitude = robot_car.longitude / 5;
                  
                
  //                  robot_car.latitude = gps.location.lat();
  //                  robot_car.longitude = gps.location.lng();
//                  robot_car.latitude = 7.007057;
//                  robot_car.longitude = 100.502220;
                  Serial.print("lat: ");
                  Serial.print(waypoint.latitude, 6);
                  Serial.print(" ");
                  Serial.print("long: ");
                  Serial.print(waypoint.longitude, 6);
                  Serial.print("\t");
                  Serial.print("RobotCarLatitude= "); 
                  Serial.print(robot_car.latitude/5, 6);
                  Serial.print(" RobotCarLongitude= "); 
                  Serial.print(robot_car.longitude/5, 6);
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
  //            }         //gps local update
          }
            
//        } // echo 
//    }         // mode
    
  }
    
}
