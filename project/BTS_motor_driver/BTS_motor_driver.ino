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


void forward() {
  Serial.println("Forward : ");
  motorA_Forward();
  motorB_Forward();
}

void backward() {
  Serial.println("Backward : ");
  motorA_Backward();
  motorB_Backward();
}

void turnRight() {
  Serial.println("Turn Right : ");
  motorA_Stop();
  motorB_Forward();
}

void turnLeft() {
  Serial.println("Turn Left : ");
  motorA_Forward();
  motorB_Stop();
}

void brake() {
  Serial.println("Brake : ");
  motorA_Stop();
  motorB_Stop();
}

//==================== motor A (right) control
void motorA_Forward() {
  analogWrite(R_PWM_A, 0);
  analogWrite(L_PWM_A, PWM);
}

void motorA_Backward() {
  analogWrite(R_PWM_A, PWM);
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

void motorB_Stop() {
  analogWrite(R_PWM_B, 0);
  analogWrite(L_PWM_B, 0);
}

void setup() {
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
  Serial.begin(9600);

}

void loop() {

  forward();
  delay(5000);
  brake();
  delay(2000);
  backward();
  delay(5000);
  brake();
  delay(2000);
//  turnRight(2000);
//  brake(3000);
//  turnLeft(2000);
//  brake(3000);
}
