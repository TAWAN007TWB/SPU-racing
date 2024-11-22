#include <ESP32Servo.h> // Using Servo library for PWM signals

Servo esc;              // Servo object to control the ESC
int motorPin = 12;      // Pin connected to the ESC signal wire

void setup() {
  // Attach ESC to the PWM pin
  esc.attach(motorPin, 1000, 2000); // PWM range: 1000-2000 microseconds
  
  // Initialize the ESC (set minimum throttle)
  esc.writeMicroseconds(1000); // Stop the motor
  delay(2000); // Allow ESC to initialize
}

void loop() {
  // Start the fan at mid-speed
  esc.writeMicroseconds(1500); // Mid-speed
  delay(5000);
  
  // Increase fan speed
  esc.writeMicroseconds(1800); // High-speed
  delay(5000);

  // Stop the fan
  stopFan();
  delay(5000); // Wait before restarting the loop
}

// Function to stop the fan
void stopFan() {
  esc.writeMicroseconds(1000); // Minimum throttle to stop the motor
}
