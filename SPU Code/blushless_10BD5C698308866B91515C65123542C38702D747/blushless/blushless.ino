#include <ESP32Servo.h> // The library can still be used to generate PWM signals

Servo esc; // Using the Servo class to send PWM signals to the ESC
int motorPin = 12; // Pin connected to the ESC signal wire

void setup() {
  // Attach ESC to the PWM pin
  esc.attach(motorPin, 1000, 2000); // PWM range for ESCs: 1000-2000 microseconds
  
  // Initialize the ESC
  esc.writeMicroseconds(1000); // Minimum throttle (motor off)
  delay(2000); // Give the ESC time to initialize
  
  // Set motor to mid-speed
  esc.writeMicroseconds(1500); // Mid throttle
  delay(5000);
  
  // Increase speed
  esc.writeMicroseconds(1800); // High throttle
  delay(5000);
  
  // Stop the motor
  esc.writeMicroseconds(1000); // Minimum throttle
}

void loop() {
  // Add your desired motor control logic here
}
