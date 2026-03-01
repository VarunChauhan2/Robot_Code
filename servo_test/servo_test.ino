#include <Servo.h>

Servo myServo;  // Create a servo object

void setup() {
  myServo.attach(9);  // Tells the Arduino the signal wire is on pin 9
}

void loop() {
  myServo.write(0);   // Move to 0 degrees
  delay(1000);        // Wait 1 second
  
  myServo.write(90);  // Move to 90 degrees (center)
  delay(1000);
  
  myServo.write(180); // Move to 180 degrees
  delay(1000);
}