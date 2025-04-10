#include <QTRSensors.h>
#include "CytronMotorDriver.h"


float Kp = 1.3; // Kp ค่าหัก ถ้า smoothเกินให้เพิ่ม ถ้าน้อยเกินให้ลด
float Ki = 0;
float Kd = 24; // Kd ความsmooth ในการหักเข้าเส้น
uint8_t maxspeeda = 230;
uint8_t maxspeedb = 230;
uint8_t basespeeda = 230;
uint8_t basespeedb = 230;
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
++
const uint8_t buttonPin = 8; // Button connected to pin D8
bool buttonPressed = false;

void setup() {
  Serial.begin(9600);
  //pinMode(buttonPin, INPUT_PULLUP); // Use internal pull-up resistor
  qtr_setup();
  //waitForButtonPress(); // Wait for the button to start moving
}

void loop() {
  PID_control();
  //qtr_show();
  //delay(200);
}
void motor(int speedL, int speedR) {
  motor1.setSpeed(speedL);
  motor2.setSpeed(speedR);
}

void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  
  int error = 3500 - position;
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


  if (sensorValues[0] < bw && sensorValues[1] < bw && sensorValues[2] < bw && sensorValues[3] < bw && sensorValues[4] < bw && sensorValues[5] < bw && sensorValues[6] < bw && sensorValues[7] < bw && state_L == 1) {
    //Serial.println("Whiteall L");
    //Serial.println(state_L);
    error = 3500;
    countstop += 2;
  } 
  else if (sensorValues[0] < bw && sensorValues[1] < bw && sensorValues[2] < bw && sensorValues[3] < bw && sensorValues[4] < bw && sensorValues[5] < bw && sensorValues[6] < bw && sensorValues[7] < bw && state_R == 1) {
    //Serial.println("Whiteall R");
    error = -3500;
    countstop += 2;
  } /*else if (sensorValues[0] > bw || sensorValues[1] > bw || sensorValues[2] > bw || sensorValues[3] > bw || sensorValues[4] > bw || sensorValues[5] > bw || sensorValues[6] > bw || sensorValues[7] > bw) {
    state_L = 0;
    state_R = 0;
    countstop = 0;
    Serial.println("inline");
  }*/

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  float motorspeed = P * Kp + I * Ki + D * Kd;

  // Serial.print("Position : ");
  //  Serial.println(position);
  //Serial.println(motorspeed);
  //delay(200);
  float motorspeeda = basespeeda + motorspeed; // Left motor
  float motorspeedb = basespeedb - motorspeed; // Right motor



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
  //Serial.println(countstop);
  if (countstop < 200) {
    motor(motorspeeda, motorspeedb);
    //Serial.println("goooooo");
  } else {
    motor(0, 0);
    //Serial.println("stopppp");
  }
  //  Serial.print("L speed");
  //  Serial.println(motorspeeda);
  //  Serial.print("R speed");
  // Serial.println(motorspeedb);
  // //
  //  delay(250);
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
