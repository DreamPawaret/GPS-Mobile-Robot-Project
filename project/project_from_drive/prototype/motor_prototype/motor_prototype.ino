#define in1PinRA 6        // right motor
#define in2PinRA 7
#define enableRight 10
#define in3PinLB 4        // left motor
#define in4PinLB 5
#define enableLeft 11

void setup() {
  pinMode(in1PinRA, OUTPUT);
  pinMode(in2PinRA, OUTPUT);
  pinMode(in3PinLB, OUTPUT);
  pinMode(in4PinLB, OUTPUT);
  pinMode(enableRight, OUTPUT);
  pinMode(enableLeft, OUTPUT);
  Serial.begin(9600);

}

void loop() {
  analogWrite(enableRight, 200);
  analogWrite(enableLeft, 200);
  forward(2000);
  coast(2000);
  turnRight(2000);
  coast(2000);
  turnLeft(2000);
  coast(2000);
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
  delay(time_val);
}

void coast(int time_val){
  motorRightCoast();
  motorLeftCoast();
  delay(time_val);
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
  digitalWrite(in1PinRA, HIGH);
  digitalWrite(in2PinRA, LOW);
}

void motorRightBackward(){
  digitalWrite(in1PinRA, LOW);
  digitalWrite(in2PinRA, HIGH);
}

// motor Left control
void motorLeftForward(){
  digitalWrite(in3PinLB, HIGH);
  digitalWrite(in4PinLB, LOW);
}

void motorLeftBackward(){
  digitalWrite(in3PinLB, LOW);
  digitalWrite(in4PinLB, HIGH);
}

void motorRightCoast(){
  digitalWrite(in1PinRA, LOW);
  digitalWrite(in2PinRA, LOW);
}

void motorLeftCoast(){
  digitalWrite(in3PinLB, LOW);
  digitalWrite(in4PinLB, LOW);
}

void motorRightBrake(){
  digitalWrite(in1PinRA, HIGH);
  digitalWrite(in2PinRA, HIGH);
}

void motorLeftBrake(){
  digitalWrite(in3PinLB, HIGH);
  digitalWrite(in4PinLB, HIGH);
}
