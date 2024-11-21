#include <QTRSensors.h>
#include "CytronMotorDriver.h"

// PID tuning parameters and robot speed limits
float Kp = 0.0001;
float Ki = 0;
float Kd = 0.001;
const uint8_t maxspeeda = 255;
const uint8_t maxspeedb = 255;
const uint8_t basespeeda = 180;
const uint8_t basespeedb = 180;
int state_L = 0;
int state_R = 0;
int countstop = 0;
int bw = 900; // Black-white threshold
int P;
int I;
int D;

int lastError = 0;
bool onoff = false;
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
CytronMD motor1(PWM_PWM, D3, D2);  // PWM 1A = Pin D3, PWM 1B = Pin D2.
CytronMD motor2(PWM_PWM, D5, D4);  // PWM 2A = Pin D5, PWM 2B = Pin D4.

const uint8_t buttonPin = 6; // Button connected to pin D6
bool buttonPressed = false;

unsigned long lastWhiteTime = 0;  // Track time when all white is seen
bool allWhiteDetected = false;  // Flag to indicate if all white has been detected

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP); // Use internal pull-up resistor
  qtr_setup();        // Run calibration routine
  waitForButtonPress(); // Wait for the button to start moving
}

void loop() {
  PID_control();
  
  if (allWhiteDetected) {
    unsigned long currentTime = millis();
    if (currentTime - lastWhiteTime >= 3000) {  // Wait for 3 seconds
      shutdownRobot();  // Shut down the robot after 3 seconds
    }
  }
}

void motor(int speedL, int speedR) {
  motor1.setSpeed(speedL);
  motor2.setSpeed(speedR);
}

void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);

  int error = 3500 - position;

  // Check sensor values and states
  if (sensorValues[3] > bw && sensorValues[4] > bw) {
    countstop = 0;
    //Serial.print("LLLLLLLLLLLLLLL " );
    //Serial.println(state_L);
  }
  
  if (sensorValues[0] > bw) {
    state_L = 1;
    state_R = 0;
    //Serial.print("LLLLLLLLLLLLLLL " );
    //Serial.println(state_L);
  }
  if (sensorValues[7] > bw) {
    state_R = 1;
    state_L = 0;
    //Serial.print("RRRRRRRRRRRRRRR ");
    //Serial.println(state_R);
  }

  // Detect all white on the left or right
  if (isAllWhite() && (state_L == 1 || state_R == 1)) {
    error = (state_L == 1) ? 3500 : -3500;
    countstop += 1;
    allWhiteDetected = true;  // Flag that all white is detected
    lastWhiteTime = millis();  // Store time when all white was first detected
  }

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;

  float motorspeed = P * Kp + I * Ki + D * Kd;
  float motorspeeda = basespeeda + motorspeed;
  float motorspeedb = basespeedb - motorspeed;

  // Limit motor speeds
  if (motorspeeda > maxspeeda) motorspeeda = maxspeeda;
  if (motorspeedb > maxspeedb) motorspeedb = maxspeedb;
  if (motorspeeda < 0) motorspeeda = 0;
  if (motorspeedb < 0) motorspeedb = 0;

  // Move robot or stop based on countstop
  if (countstop < 200) {
    motor(motorspeeda, motorspeedb);
    //Serial.println("goooooo");
  } else {
    motor(0, 0);
    //Serial.println("stopppp");
  }

  // Debugging outputs
  Serial.print("Sensors: ");
  for (int i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  Serial.println();
  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("Countstop: ");
  Serial.println(countstop);
}

bool isAllWhite() {
  // Check if all sensors detect white (values below threshold)
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > bw) {
      return false;  // At least one sensor is not detecting white
    }
  }
  return true;  // All sensors are detecting white
}

void qtr_setup() {
  Serial.println("Calibrating sensors...");
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5, A6, A7 }, SensorCount);
  qtr.setEmitterPin(12);
  delay(500);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, LOW);
  
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();

    // Optional: Print raw sensor values for monitoring during calibration
    if (i % 50 == 0) {
      for (uint8_t j = 0; j < SensorCount; j++) {
        Serial.print(sensorValues[j]);
        Serial.print("\t");
      }
      Serial.println();
    }
  }

  digitalWrite(LED_BLUE, HIGH);
  Serial.println("Calibration complete!");

  // Debugging calibration outputs
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print("Min: ");
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(", Max: ");
    Serial.println(qtr.calibrationOn.maximum[i]);
  }
  delay(1000);
}

void waitForButtonPress() {
  Serial.println("Press the button to start...");
  while (digitalRead(buttonPin) == HIGH) {
    // Wait for the button to be pressed (LOW signal due to pull-up).
  }
  Serial.println("Button pressed! Starting...");
}

void shutdownRobot() {
  Serial.println("Shutting down robot...");
  motor(0, 0);  // Stop motors
  while (true) {
    // Keep robot stopped indefinitely
  }
}
