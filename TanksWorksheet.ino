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
  driveMotor(1, 255);
  driveMotor(2, 255);
  driveMotor(3, 255);
  driveMotor(4, 255);
  delay(1000);

  driveMotor(1, -255);
  driveMotor(2, -255);
  driveMotor(3, -255);
  driveMotor(4, -255);
  delay(1000);

  driveMotor(1, 0);
  driveMotor(2, 0);
  driveMotor(3, 0);
  driveMotor(4, 0);
  delay(1000);
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
