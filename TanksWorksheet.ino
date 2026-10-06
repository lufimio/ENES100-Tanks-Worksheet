#include "Tank.h"

#define PIN_MOTOR_1_FORWARD 11
#define PIN_MOTOR_1_REVERSE 10
#define PIN_MOTOR_2_FORWARD 9
#define PIN_MOTOR_2_REVERSE 8
#define PIN_MOTOR_3_FORWARD 4
#define PIN_MOTOR_3_REVERSE 5
#define PIN_MOTOR_4_FORWARD 2
#define PIN_MOTOR_4_REVERSE 3
// Bump Sensors
#define PIN_BUMP_FRONT 32
#define PIN_BUMP_REAR 33
// Ultrasonic Sensors
#define PIN_US_FRONT_TRIG 18
#define PIN_US_FRONT_ECHO 19
#define PIN_US_REAR_TRIG 27
#define PIN_US_REAR_ECHO 26
// Infrared Sensors
#define PIN_IR_1 15
#define PIN_IR_2 17
#define PIN_IR_3 14
#define PIN_IR_4 16
// WiFi Modules
#define PIN_WIFI_TX 52
#define PIN_WIFI_RX 50

void setup() {
  pinMode(PIN_MOTOR_1_FORWARD, OUTPUT);
  pinMode(PIN_MOTOR_1_REVERSE, OUTPUT);
  pinMode(PIN_MOTOR_2_FORWARD, OUTPUT);
  pinMode(PIN_MOTOR_2_REVERSE, OUTPUT);
  pinMode(PIN_MOTOR_3_FORWARD, OUTPUT);
  pinMode(PIN_MOTOR_3_REVERSE, OUTPUT);
  pinMode(PIN_MOTOR_4_FORWARD, OUTPUT);
  pinMode(PIN_MOTOR_4_REVERSE, OUTPUT);
}

void loop() {
  driveMecanum(255, 0, 0);
  delay(1000);
  driveMecanum(0, 255, 0);
  delay(1000);
  driveMecanum(0, 0, 255);
  delay(1000);
  driveMecanum(0, 0, 0);
  delay(500);
  driveMecanum(255, 255, 255);
  delay(1000);
  driveMecanum(255, 255, 0);
  delay(1000);
  driveMecanum(0, 0, 0);
  delay(1000);
}

void driveMecanum(int fwdPWD, int strPWD, int trnPWD) {
  int M1Vel = fwdPWD + strPWD - trnPWD;
  int M2Vel = fwdPWD - strPWD - trnPWD;
  int M3Vel = fwdPWD - strPWD + trnPWD;
  int M4Vel = fwdPWD + strPWD + trnPWD;

  float maxVel = max(max(M1Vel, M2Vel), max(M3Vel, M4Vel));

  if (maxVel > 255) {
    M1Vel = (int) (255.0 * (float) M1Vel / maxVel);
    M2Vel = (int) (255.0 * (float) M2Vel / maxVel);
    M3Vel = (int) (255.0 * (float) M3Vel / maxVel);
    M4Vel = (int) (255.0 * (float) M4Vel / maxVel);
  }

  driveMotor(1, M1Vel);
  driveMotor(2, M2Vel);
  driveMotor(3, M3Vel);
  driveMotor(4, M4Vel);
}

void driveMotor(int motor, int pwm) {
  int forwardPin, backwardPin;
  if (motor == 1) {
    forwardPin = PIN_MOTOR_1_FORWARD;
    backwardPin = PIN_MOTOR_1_REVERSE;
  } else if (motor == 2) {
    forwardPin = PIN_MOTOR_2_FORWARD;
    backwardPin = PIN_MOTOR_2_REVERSE;
  } else if (motor == 3) {
    forwardPin = PIN_MOTOR_3_FORWARD;
    backwardPin = PIN_MOTOR_3_REVERSE;
  } else if (motor == 4) {
    forwardPin = PIN_MOTOR_4_FORWARD;
    backwardPin = PIN_MOTOR_4_REVERSE;
  } else return;
  
  if (pwm >= 0) {
    analogWrite(forwardPin, pwm);
    analogWrite(backwardPin, 0);
  } else {
    analogWrite(forwardPin, 0);
    analogWrite(backwardPin, -pwm);
  }
}
