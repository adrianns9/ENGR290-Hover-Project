#include <Arduino.h>
#include <DistanceGP2Y0A21YK.h>
#include <HCSR04.h>
#include <MPU6050.h>
#include <Servo.h>
#include <StateMachine.h>
#include <Wire.h>
#include <neotimer.h>

// Threshold Values
#define TURN_TRIGGER_DIST 5
#define TURN_SIDE_DIST 25
#define MIN_TIME_TO_TURN 5000 // ms
#define SERVO_MIN_MAX 30

// Pin Configuration
#define SERVO_PIN 5          // CONNECT TO PORT P3
#define THRUST_FAN_PIN 9     // PORT P9
#define LEVITATION_FAN_PIN 6 // PORT P4
#define IR_SENSOR_PIN A0     // PORT P5
#define US_TRIGGER_PIN 11    // PORT P6
#define US_ECHO_PIN 2

MPU6050 mpu;
Servo servo;

// Fan Control
void setThrustSpeed(uint8_t speed) { analogWrite(THRUST_FAN_PIN, speed); }

void setLevitationSpeed(uint8_t speed) {
  analogWrite(LEVITATION_FAN_PIN, speed);
}

// State Machine
void StraightPathState();
boolean CheckIfOnTurn();
void TurnState();
boolean CheckIfOnStraight();

enum HOVER_STATE { STRAIGHT, LEFT, RIGHT };
StateMachine machine = StateMachine();
State *S0 = machine.addState(&StraightPathState);
State *S1 = machine.addState(&TurnState);

// Sensors
UltraSonicDistanceSensor frontSensor(US_TRIGGER_PIN, US_ECHO_PIN);
DistanceGP2Y0A21YK sideSensor;

// Timers
Neotimer usTimer = Neotimer(15);
Neotimer irTimer = Neotimer(50);
Neotimer logTimer = Neotimer(500);
Neotimer turnTimer = Neotimer(MIN_TIME_TO_TURN);
volatile uint32_t prevTime;

const double sensFacGyro = 131.0 * 1000.0;
const double sensFacAccel = 8192.0 * 1000.0;
volatile int16_t velX{0}, velY{0}, velZ{0};
volatile int64_t curSpeed{0};
double curAngle = 90;
double initAngle = curAngle;
double targetAngle = curAngle;
volatile int32_t rawCurAngle = curAngle * sensFacGyro;
volatile uint8_t servoPos = 90;
volatile float distWall, distFront;

void setup() {
  using namespace MPU6050_IMU;
  pinMode(A5, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);

  Wire.begin();
  Wire.setWireTimeout(3000, true);

#if DEBUG
  Serial.begin(9600);
#endif

  S0->addTransition(&CheckIfOnTurn, S1);
  S1->addTransition(&CheckIfOnStraight, S0);

  // Initialize MPU6050
  mpu.setTempSensorEnabled(false);
  mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setSleepEnabled(false);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful")
                                      : F("MPU6050 connection failed"));

  // Calibration
  mpu.CalibrateGyro();
  mpu.CalibrateAccel();
  mpu.PrintActiveOffsets();

  // Servo Setup
  servo.attach(SERVO_PIN);
  servo.write(servoPos);

  // Fan Setup
  pinMode(LEVITATION_FAN_PIN, OUTPUT);
  pinMode(THRUST_FAN_PIN, OUTPUT);

  // IR Sensor Setup
  sideSensor.begin(IR_SENSOR_PIN);

  // Initialize Timers
  prevTime = millis();
}

void loop() {
  setLevitationSpeed(255);
  setThrustSpeed(255);
  int16_t gyroX, gyroY, gyroZ;
  int16_t accelX, accelY, accelZ;

  uint32_t curTime = millis();
  uint32_t dt = curTime - prevTime;

  double curD;
  if (dt > 15) {
    mpu.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
    rawCurAngle += dt * gyroZ;
    curD = rawCurAngle / sensFacGyro;

    velX += dt * accelX;
    velY += dt * accelY;
    velZ += dt * accelZ;

    double serveA = 90.0 - (targetAngle - curD);
    uint8_t servoAngle = constrain(serveA, SERVO_MIN_MAX, 180 - SERVO_MIN_MAX);
    servo.write(servoAngle);
    Serial.println(curD);

    prevTime = curTime;
  }

  if (usTimer.repeat()) {
    distFront = frontSensor.measureDistanceCm();
  }

  if (irTimer.repeat()) {
    distWall = sideSensor.getDistanceCentimeter();
  }

#if DEBUG
  if (logTimer.repeat()) {
    // Serial.println("Print LINE");
    // Serial.println(rawCurAngle);
    // Serial.println(targetAngle);
    // Serial.println(curD);
    // Serial.print("Cur Angle: ");
    // Serial.println(curD);
    // Serial.print("Dist Front: ");
    // Serial.println(distFront);
    // Serial.print("Dist Wall: ");
    // Serial.println(distWall);

    // double pVelX = velX / sensFacAccel;
    // double pVelY = velY / sensFacAccel;
    // double pVelZ = velZ / sensFacAccel;
    // Serial.print("Velocity: ");
    // Serial.println(pVelX);
    // Serial.println(pVelY);
    // Serial.println(pVelZ);
    // Serial.println("");
  }
#endif

  machine.run();
}

void lockTargetAngle(double angle) { targetAngle = angle; }

void StraightPathState() {
#if DEBUG
  if (logTimer.repeat()) {
    Serial.println("State: Straight");
  }
#endif
}

boolean CheckIfOnTurn() {
  // static volatile uint8_t count = 0;
  if (TURN_TRIGGER_DIST > distFront && distFront > 0) {
    Serial.println("TRIGGER");
    turnTimer.start();
    double deltaAngle = (TURN_SIDE_DIST > distWall) ? 90.0 : -90.0;
    lockTargetAngle(targetAngle - deltaAngle);
    return true;
  }
  return false;
}

void TurnState() {
  // Serial.print("Turn State: ");
  // Serial.println(distFront);
}

boolean CheckIfOnStraight() {
  // static volatile uint8_t count = 0;
  if (TURN_TRIGGER_DIST < distFront && turnTimer.done()) {
    turnTimer.reset();
    Serial.println("TRIGGER");
    return true;
  }
  return false;
}
