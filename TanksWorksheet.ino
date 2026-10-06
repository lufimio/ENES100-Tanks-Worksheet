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

int direction = 1;

void setup() {
    // Motors
    pinMode(PIN_MOTOR_1_FORWARD, OUTPUT);
    pinMode(PIN_MOTOR_1_REVERSE, OUTPUT);
    pinMode(PIN_MOTOR_2_FORWARD, OUTPUT);
    pinMode(PIN_MOTOR_2_REVERSE, OUTPUT);
    pinMode(PIN_MOTOR_3_FORWARD, OUTPUT);
    pinMode(PIN_MOTOR_3_REVERSE, OUTPUT);
    pinMode(PIN_MOTOR_4_FORWARD, OUTPUT);
    pinMode(PIN_MOTOR_4_REVERSE, OUTPUT);

    // Bump Sensors
    pinMode(PIN_BUMP_FRONT, INPUT);
    pinMode(PIN_BUMP_REAR, INPUT);

    // Ultrasonic Sensors
    pinMode(PIN_US_FRONT_TRIG, OUTPUT);
    pinMode(PIN_US_FRONT_ECHO, INPUT);
    pinMode(PIN_US_REAR_TRIG, OUTPUT);
    pinMode(PIN_US_REAR_ECHO, INPUT);

    // Infrared Sensors
    pinMode(PIN_IR_1, INPUT);
    pinMode(PIN_IR_2, INPUT);
    pinMode(PIN_IR_3, INPUT);
    pinMode(PIN_IR_4, INPUT);
}

void loop() {
  driveMecanum(100, 0, 0);
  
  if (readFrontBumpSwitch()) {
    driveMecanum(-100, 0, 0);
    delay(3000);
    driveMecanum(0, 0, 255);
    delay(1800);
  }
}

void driveMecanum(int fwdPWD, int strPWD, int trnPWD) {
  int M1Vel = fwdPWD + strPWD + trnPWD;
  int M2Vel = fwdPWD - strPWD + trnPWD;
  int M3Vel = fwdPWD - strPWD - trnPWD;
  int M4Vel = fwdPWD + strPWD - trnPWD;

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

  pwm = max(pwm, -255);
  pwm = min(pwm, 255);
 
  if (pwm >= 0) {
    analogWrite(forwardPin, abs(pwm));
    analogWrite(backwardPin, 0);
  } else {
    analogWrite(forwardPin, 0);
    analogWrite(backwardPin, abs(pwm));
  }
}

bool readFrontBumpSwitch() {
  return digitalRead(PIN_BUMP_FRONT);
}

bool readRearBumpSwitch() {
  return digitalRead(PIN_BUMP_REAR);
}

float readFrontDistanceSensor() {
    digitalWrite(PIN_US_FRONT_TRIG, 0);
    delayMicroseconds(2);
    digitalWrite(PIN_US_FRONT_TRIG, 1);
    delayMicroseconds(10);
    digitalWrite(PIN_US_FRONT_TRIG, 0);

    int duration = pulseIn(PIN_US_FRONT_ECHO, 1);
    if (duration < 20000 && duration > 0) {
        return min(duration * .0001715, 1);
    } else return -1;
}


float readRearDistanceSensor() {
    digitalWrite(PIN_US_REAR_TRIG, 0);
    delayMicroseconds(2);
    digitalWrite(PIN_US_REAR_TRIG, 1);
    delayMicroseconds(10);
    digitalWrite(PIN_US_REAR_TRIG, 0);

    int duration = pulseIn(PIN_US_REAR_ECHO, 1);
    if (duration < 20000 && duration > 0) {
        return min(duration * .0001715, 1);
    } else return -1;
}
