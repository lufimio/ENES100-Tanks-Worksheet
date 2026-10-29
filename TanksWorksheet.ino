#include "Enes100.h"

#define PI 3.14159265358979323

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

float lastKnownTheta = 0;
float lastKnownX = 0;
float lastKnownY = 0;
long lastUpdateMillis = 0;

const float wheelDiameter = 0;

const float maxAngularVelocity = 0.87;
const float correctionFactor = 1;

const int minTurnPWM = 50;
const int maxTurnPWM = 255;
const float slowdownAngle = 0.8;
const float slowdownDistance = 0.5;

const unsigned long maxTurnTime = 2000;
const unsigned long trackingTimeout = 2000;

bool firstRunDone = false;

void setup() {
  Enes100.begin("Team 4", HYDROGEN, 15, 1116, PIN_WIFI_TX, PIN_WIFI_RX);

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
  if (!firstRunDone) {
    waitForTracking();
    main()
    firstRunDone = true;
  }
}

void task2() {
  Enes100.println("turning to 90 degrees")
  turnToAngle(toRadians(90));
  delay(3000);
  Enes100.println("turning to 180 degrees")
  turnToAngle(toRadians(180));
  delay(3000);
  Enes100.println("turning to 90 degrees")
  turnToAngle(toRadians(90));
  delay(3000);
  Enes100.println("turning to 0 degrees")
  turnToAngle(toRadians(0));
  delay(3000);
}

void main() {
  float startingAngle;

  if (getY() < 1) {
    startingAngle = toRadians(90);
  } else {
    startingAngle = toRadians(-90);
  }

  turnToAngle(startingAngle);

  while (!readFrontBumpSwitch()) {
    driveMecanum(255, 0, 0);
    delay(50);
  }
  stopDrive();

  // do processing tasks

  while(!readRearBumpSwitch()) {
    driveMecanum(-255, 0, 0);
    delay(50);
  }
  stopDrive();

  while (getX() < 2.75) {
    driveMecanum(-10, 255, 0);
    delay(50);
  }

  driveToPoint(3, 1.5, 0.1);
  driveToPoint(3.75, 1.5, 0.3);
}

void turnToAngle(float angle, float tolerance = 0.08) {
  unsigned long startTime = millis();

  while (millis() - startTime < 10000) {
    unsigned long previousUpdate = lastUpdateMillis;
    unsigned long waitStart = millis();

    while (lastUpdateMillis == previousUpdate) {
      updateInfo();

      if (millis() - waitStart >= trackingTimeout) {
        stopDrive();
        return;
      }

      delay(10);
    }

    float delta = normalizeAngle(getTheta() - angle);
    float error = abs(delta);
    Enes100.println(delta);

    if (error < tolerance)
      break;

    float speedFactor = constrain(error / slowdownAngle, 0.0f, 1.0f);
    int pwm = minTurnPWM + (int)((maxTurnPWM - minTurnPWM) * speedFactor);
    float angularVelocity = maxAngularVelocity * ((float)pwm / maxTurnPWM);
    unsigned long duration = (unsigned long)(error / angularVelocity * 1000.0f * correctionFactor);
    duration = min(duration, maxTurnTime);

    driveMecanum(0, 0, delta > 0 ? pwm : -pwm);
    delay(duration);
    stopDrive();

    delay(100);
  }

  stopDrive();
}

void driveToPoint(float targetX, float targetY, float tolerance = 0.08) {
  unsigned long startTime = millis();

  while (millis() - startTime < 10000) {
    updateInfo();

    if (millis() - lastUpdateMillis > 10000) {
      stopDrive();
      return;
    }

    float dx = targetX - getX();
    float dy = targetY - getY();

    float distance = sqrtf(dx * dx + dy * dy);

    if (distance < tolerance) {
      stopDrive();
      return;
    }

    float theta = getTheta();

    float forward = dx * cosf(theta) + dy * sinf(theta);
    float strafe = -dx * sinf(theta) + dy * cosf(theta);

    float speedFactor = constrain(
      distance / slowdownDistance,
      0.0f,
      1.0f
    );

    int pwm = minTurnPWM +
      (int)((maxTurnPWM - minTurnPWM) * speedFactor);

    int forwardPWM = (int)(forward / distance * pwm);
    int strafePWM = (int)(strafe / distance * pwm);

    int maxPWM = max(abs(forwardPWM), abs(strafePWM));

    if (maxPWM > 255) {
      forwardPWM = (int)(forwardPWM * 255.0f / maxPWM);
      strafePWM = (int)(strafePWM * 255.0f / maxPWM);
    }

    driveMecanum(forwardPWM, strafePWM, 0);

    delay(50);
  }

  stopDrive();
}

bool waitForTracking() {
  unsigned long startTime = millis();

  while (millis() - startTime < trackingTimeout) {
    updateInfo();
    if (millis() - lastUpdateMillis < 200)
      return true;
    delay(10);
  }

  return false;
}

float normalizeAngle(float angle) {
  angle = fmodf(angle + PI, 2 * PI);
  if (angle < 0)
    angle += 2 * PI;
  return angle - PI;
}

float toRadians(float deg) {
  return deg * PI / 180.0;
}

float toDegrees(float rad) {
  return rad * 180.0 / PI;
}

void updateInfo() {
  getX();
  getY();
  getTheta();
}

float getX() {
  float x = Enes100.getX()
  if (x != -1.0f) {
    lastKnownX = Enes100.getX();
    lastUpdateMillis = millis();
  }
  return lastKnownX;
}

float getY() {
  float y = Enes100.getY()
  if (y != -1.0f) {
    lastKnownY = Enes100.getY();
    lastUpdateMillis = millis();
  }
  return lastKnownY;
}

float getTheta() {
  float theta = Enes100.getTheta();
  if (theta != -1.0f) {
    lastKnownTheta = Enes100.getTheta();
    lastUpdateMillis = millis();
  }
  return lastKnownTheta;
}

void stopDrive() {
  driveMecanum(0, 0, 0);
}

void driveMecanum(int fwdPWD, int strPWD, int trnPWD) {
  int M1Vel = fwdPWD + strPWD + trnPWD;
  int M2Vel = fwdPWD - strPWD + trnPWD;
  int M3Vel = fwdPWD - strPWD - trnPWD;
  int M4Vel = fwdPWD + strPWD - trnPWD;

  int maxVel = max(
    max(abs(M1Vel), abs(M2Vel)),
    max(abs(M3Vel), abs(M4Vel))
  );

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
