#include <QTRSensors.h>
#include "CytronMotorDriver.h"

float Kp = 1.2; // Kp ค่าหัก ถ้า smoothเกินให้เพิ่ม ถ้าน้อยเกินให้ลด
float Ki = 0;
float Kd = 28; // Kd ความsmooth ในการหักเข้าเส้น
uint8_t maxspeeda = 255;  // Removed const to allow changes
uint8_t maxspeedb = 255;  // Removed const to allow changes
const uint8_t basespeeda = 255;
const uint8_t basespeedb = 255;
int state_L = 0;
int state_R = 0;
int countstop = 0;
int bw = 900;
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
    maxspeeda /= 2; // Reduce max speed by half
    maxspeedb /= 2;
    isSlow = true;
    Serial.println("All Black Detected: Slowing Down...");
    delay(1000); // Small pause for stabilization
    return; // Wait until condition changes
  }

  if (allBlack && isSlow) {
    // If all black detected again, restore max speed
    maxspeeda *= 2; // Restore max speed
    maxspeedb *= 2;
    isSlow = false;
    Serial.println("All Black Detected Again: Restoring Speed...");
    delay(1000); // Small pause for stabilization
    return; // Wait until condition changes
  }

  if (sensorValues[3] > bw && sensorValues[4] > bw) {
    countstop = 0;
  }
  if (sensorValues[0] > bw) {
    state_L = 1;
    state_R = 0;
  }
  if (sensorValues[7] > bw) {
    state_R = 1;
    state_L = 0;
  }

  if (sensorValues[0] < bw && sensorValues[1] < bw && sensorValues[2] < bw && sensorValues[3] < bw &&
      sensorValues[4] < bw && sensorValues[5] < bw && sensorValues[6] < bw && sensorValues[7] < bw && state_L == 1) {
    error = 3500;
    countstop += 2;
  } else if (sensorValues[0] < bw && sensorValues[1] < bw && sensorValues[2] < bw && sensorValues[3] < bw &&
             sensorValues[4] < bw && sensorValues[5] < bw && sensorValues[6] < bw && sensorValues[7] < bw && state_R == 1) {
    error = -3500;
    countstop += 2;
  }

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  float motorspeed = P * Kp + I * Ki + D * Kd;

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

  if (countstop < 200) {
    motor(motorspeeda, motorspeedb);
  } else {
    motor(0, 0);
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

void qtr_show() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  delay(1);
}

void waitForButtonPress() {
  Serial.println("Press the button to start...");
  while (digitalRead(buttonPin) == HIGH) {
    // Wait for the button to be pressed (LOW signal due to pull-up).
  }
  Serial.println("Button pressed! Starting...");
}
