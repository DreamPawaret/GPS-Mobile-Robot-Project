#define in1PinR 2
#define in2PinR 3
#define in3PinL 4
#define in4PinL 5
#define enableRight 10
#define enableLeft 11 

void setup() {
  pinMode(in1PinR, OUTPUT);
  pinMode(in2PinR, OUTPUT);
  pinMode(in3PinL, OUTPUT);
  pinMode(in4PinL, OUTPUT);
  pinMode(enableRight, OUTPUT);
  pinMode(enableLeft, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  
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
  motorRightBackward();
  motorLeftForward();
  delay(time_val);
}

void turnLeft(int time_val){
  motorRightForward();
  motorLeftBackward();
}

void uturn(int time_val){
  motorRightForward();
  motorLeftBackward();
  delay(time_val);
}

void brake(int time_val){
  motorRightBrake();
  motorLeftBrake();
  delay(time_val);
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

// motor Right control
void motorRightForward(){
  digitalWrite(in1PinR, HIGH);
  digitalWrite(in2PinR, LOW);
}

void motorRightBackward(){
  digitalWrite(in1PinR, LOW);
  digitalWrite(in2PinR, HIGH);
}

// motor Left control
void motorLeftForward(){
  digitalWrite(in3PinL, HIGH);
  digitalWrite(in4PinL, LOW);
}

void motorLeftBackward(){
  digitalWrite(in3PinL, LOW);
  digitalWrite(in4PinL, HIGH);
}

void motorRightBrake(){
  digitalWrite(in1PinR, HIGH);
  digitalWrite(in2PinR, HIGH);
}

void motorLeftBrake(){
  digitalWrite(in3PinL, HIGH);
  digitalWrite(in4PinL, HIGH);
}
