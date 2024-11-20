#include <ESP32Servo.h>
Servo myservo;
int servoPin = 13;
void setup() {
  // put your setup code here, to run once:
  myservo.attach(13);
  myservo.write(90);
  delay(5000);
  myservo.write(150);
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:

}
