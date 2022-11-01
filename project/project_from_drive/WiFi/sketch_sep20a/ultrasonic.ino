//Example By ArduinoAll
const int echo = 13;
int trig = 12;
void setup() {
  Serial.begin(9600);
}
void loop()
{
  long duration, cm;
  pinMode(echo, OUTPUT);
  digitalWrite(echo, 0);
  delayMicroseconds(2);
  digitalWrite(echo, 1);
  delayMicroseconds(5);
  digitalWrite(echo, 0);
  pinMode(trig, INPUT);
  duration = pulseIn(trig, 1);
  cm = microsecondsToCentimeters(duration);
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  delay(100);
}
long microsecondsToCentimeters(long microseconds)
{
  // ความเร็วเสียงเดินทางในอากาศคือ 340 เมตร/วินาที หรือ 29 ไมโครวินาที ต่อเซนติเมตร
  // เวลาที่ Ping ออกไปกระทบวัตถุจนถึงกลับมาที่ตัวรับ ใช้หาระยะทางได้
  // วัตถุอยู่มีระยะทางเท่ากับครึ่งหนึ่งของเวลาที่จับได้
  // เขียนเป็นสมการคำนวนระยะทาง ได้ดังนี้
  return microseconds / 29 / 2;
}
