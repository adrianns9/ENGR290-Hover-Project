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
#define MIN_TIME_TO_TURN 5000 // ms

// Pin Configuration
#define SERVO_PIN 9          // CONNECT TO PORT P9
#define THRUST_FAN_PIN 5     // PORT P3
#define LEVITATION_FAN_PIN 6 // PORT P4
#define IR_SENSOR_PIN 14     // PORT P5
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

enum curState { STRAIGHT, LEFT, RIGHT };

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
volatile int64_t curSpeed{0};
volatile int32_t curAngle = 0;
volatile int32_t initAngle = curAngle;
volatile uint8_t servoPos = 90;
volatile float distWall, distFront;
const double sensFac = 131.0 * 1000.0;

void setup() {
  using namespace MPU6050_IMU;

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

  // Calibration
  // mpu.CalibrateGyro();
  // mpu.CalibrateAccel();
  mpu.setZGyroOffset(3);
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

  if (dt > 15) {
    mpu.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
    curAngle += dt * gyroZ;

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
    double curD = curAngle / sensFac;
    servo.write(curD);

    Serial.println("Print LINE");
    Serial.print("Cur Angle: ");
    Serial.println(curD);
    Serial.print("Dist Front: ");
    Serial.println(distFront);
    Serial.print("Dist Wall: ");
    Serial.println(distWall);
  }
#endif

  machine.run();
}

void LockAngle() {}

void StraightPathState() {
  // Serial.print("Straight State: ");
  // Serial.println(distFront);
}

boolean CheckIfOnTurn() {
  // static volatile uint8_t count = 0;
  if (TURN_TRIGGER_DIST > distFront && distFront > 0) {
    Serial.println("TRIGGER");
    turnTimer.start();
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
