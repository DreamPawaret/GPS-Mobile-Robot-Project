#define INTERVAL_MESSAGE1 5000 //5วินาที
#define INTERVAL_MESSAGE2 7000 //7วินาที
#define INTERVAL_MESSAGE3 11000 //11วินาที
#define INTERVAL_MESSAGE4 13000 //13วินาที
 
unsigned long time_1 = 0;
unsigned long time_2 = 0;
unsigned long time_3 = 0;
unsigned long time_4 = 0;
 
void print_time(unsigned long time_millis);
 
void setup() {
    Serial.begin(9600);
}
 
void loop() {
//  Serial.println("hello 1");
//  delay(2000);
//  Serial.println("hello 2");
//  delay(5000);

    if(millis() - time_1 > INTERVAL_MESSAGE1){
        time_1 = millis();
        print_time(time_1);
        Serial.println("I'm message number one!");
    }
   
    if(millis() - time_2 > INTERVAL_MESSAGE2){
        time_2 = millis();
        print_time(time_2);
        Serial.println("Hello, I'm the second message.");
    }
   
    if(millis() - time_3 > INTERVAL_MESSAGE3){
        time_3 = millis();
        print_time(time_3);
        Serial.println("My name is Message the third.");
    }
   
    if(millis() - time_4 > INTERVAL_MESSAGE4){
        time_4 = millis();
        print_time(time_4);
        Serial.println("Message four is in the house!");
    }

}
 
void print_time(unsigned long time_millis){
    Serial.print("Time: ");
    Serial.print(time_millis/1000);
    Serial.print("s - ");
}
