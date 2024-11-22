#include <QTRSensors.h>
#include "CytronMotorDriver.h"

float Kp = 1.2; // Kp for correction (increase for sharper turn, decrease for smoother turn)
float Ki = 0;
float Kd = 28; // Kd for smoothing the turn
uint8_t maxspeeda = 255; // Regular variables to modify speed
uint8_t maxspeedb = 255;
const uint8_t basespeeda = 255;
const uint8_t basespeedb = 255;
int state_L = 0;
int state_R = 0;
int countstop = 0;
int bw = 800;
int P;
int I;
int D;

int lastError = 0;
bool onoff = false;
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
CytronMD motor1(PWM_PWM, D2, D3);  // PWM 1A = Pin D3, PWM 1B = Pin D2.
CytronMD motor2(PWM_PWM, D4, D5);  // PWM 2A = Pin D5, PWM 2B = Pin D4.

const uint8_t buttonPin = 8; // Button connected to pin D8
bool buttonPressed = false;

void setup() {
  Serial.begin(9600);
  qtr_setup();
}

void loop() {
  PID_control();
}

void motor(int speedL, int speedR) {
  motor1.setSpeed(speedL);
  motor2.setSpeed(speedR);
}

void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = 3500 - position;

  // Check if all sensors detect black
  bool allBlack = true;
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] <= bw) {
      allBlack = false;
      break;
    }
  }

  // Handle speed adjustment based on "all black" detection
  static bool isSlow = false; // Tracks whether the robot is in slow mode
  if (allBlack && !isSlow) {
    // If all black detected for the first time, slow down
    Serial.println("All black detected, slowing down...");
    maxspeeda /= 2; // Reduce max speed by half
    maxspeedb /= 2;
    isSlow = true;
    delay(1000); // Small pause for stabilization
    return; // Wait until condition changes
  }

  if (allBlack && isSlow) {
    // If all black detected again, restore max speed
    Serial.println("All black detected again, restoring speed...");
    maxspeeda *= 2; // Restore max speed
    maxspeedb *= 2;
    isSlow = false;
    delay(1000); // Small pause for stabilization
    return; // Wait until condition changes
  }

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  float motorspeed = P * Kp + I * Ki + D * Kd;

  // Adjust motor speed based on PID values
  float motorspeeda = basespeeda + motorspeed; // Left motor
  float motorspeedb = basespeedb - motorspeed; // Right motor

  // Clamp motor speeds to max allowed speed
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  }

  // Control the motors based on the countstop value
  if (countstop < 200) {
    motor(motorspeeda, motorspeedb);
  } else {
    motor(0, 0); // Stop if too many consecutive "white all" readings
  }
}

void qtr_setup() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A7, A6, A5, A4, A3, A2, A1, A0 }, SensorCount);
  qtr.setEmitterPin(11);
  delay(500);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, LOW);
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BLUE, HIGH);
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  delay(1000);
}

void waitForButtonPress() {
  Serial.println("Press the button to start...");
  while (digitalRead(buttonPin) == HIGH) {
    // Wait for the button to be pressed (LOW signal due to pull-up).
  }
  Serial.println("Button pressed! Starting...");
}
