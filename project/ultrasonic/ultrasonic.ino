int echoPinL = 2;
int trigPinL = 31;
int echoPinF = 32;
int trigPinF = 33;
int echoPinR = 34;
int trigPinR = 35;

long uDuration; 
int uDistance;

void setup() {
  pinMode(trigPinL, OUTPUT); 
  pinMode(echoPinL, INPUT);
  pinMode(trigPinF, OUTPUT); 
  pinMode(echoPinF, INPUT); 
  pinMode(trigPinR, OUTPUT); 
  pinMode(echoPinR, INPUT);  
  Serial.begin(9600); 
}

void loop() {
  long SensorL = getLength(echoPinL, trigPinL);
  long SensorF = getLength(echoPinF, trigPinF);
  long SensorR = getLength(echoPinR, trigPinR);


  Serial.print("L: ");
  Serial.print(SensorL);
  Serial.print("\tFR: ");
  Serial.print(SensorF);
  Serial.print("\tR: ");
  Serial.print(SensorR);
  Serial.println(" cm ");
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
