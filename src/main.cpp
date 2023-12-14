#include <Arduino.h>
#include <DistanceGP2Y0A21YK.h>
#include <HCSR04.h>
#include <IR_Sensor.h>
#include <MPU6050.h>
#include <PID_v1.h>
#include <Servo.h>
#include <StateMachine.h>
#include <Wire.h>
#include <neotimer.h>

// Threshold Values
#define TURN_TRIGGER_DIST 55
#define CORRECT_DIST_FRONT 55
#define CHECK_IF_STRAIGHT_DIST 30
#define TURN_SIDE_DIST 35
#define MIN_TIME_TO_TURN 400 // ms
#define SERVO_MIN_MAX 15     // degrees
#define V_REF 4.0
#define ANGLE_TURN_THRESH 27 // degrees

// Speed Values
#define THRUST_MAX_SPEED 255
#define LEVITATION_MAX_SPEED 255

// Pin Configuration
#define SERVO_PIN 9          // CONNECT TO PORT P9
#define THRUST_FAN_PIN 5     // PORT P3
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
void Corrective90Wall();
boolean CheckIfOnTurn();

void TurnState();
boolean CheckIfNeeds90Correct();
boolean CheckIfOnStraight();

enum HOVER_STATE { STRAIGHT, LEFT, RIGHT };
StateMachine machine = StateMachine();
State *S0 = machine.addState(&StraightPathState);
State *S1 = machine.addState(&TurnState);
State *S2 = machine.addState(&Corrective90Wall);

// Sensors
UltraSonicDistanceSensor frontSensor(US_TRIGGER_PIN, US_ECHO_PIN);

// Timers
Neotimer usTimer = Neotimer(15);
Neotimer irTimer = Neotimer(50);
Neotimer logTimer = Neotimer(1000);
Neotimer logStateTimer = Neotimer(1000);
Neotimer turnTimer = Neotimer(MIN_TIME_TO_TURN);
Neotimer correctTimer = Neotimer(1500);
Neotimer slowdownTimer = Neotimer(500);
Neotimer correctThrustTimer = Neotimer(100);
Neotimer preSlowTimer = Neotimer(400);
volatile uint32_t prevTime;

const double sensFacGyro = 131.0 * 1000.0;
const double sensFacAccel = 8192.0 * 1000.0;
volatile int16_t rawVelX{0}, rawVelY{0}, rawVelZ{0};
volatile float distWall, distFront;

double curSpeed = 0;
double curAngle = 90;
double initAngle = curAngle;
double targetAngle = curAngle;
volatile int32_t rawCurAngle = curAngle * sensFacGyro;
volatile uint8_t servoPos = 90;
double deltaServo = 0;

// PID Controller
double Kp = 2, Ki = 5, Kd = 1;
PID servoPID(&curAngle, &deltaServo, &targetAngle, Kp, Ki, Kd, DIRECT);

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
  S1->addTransition(&CheckIfNeeds90Correct, S2);

  S0->addTransition(&CheckIfNeeds90Correct, S2);
  S2->addTransition(&CheckIfOnStraight, S0);

  // Initialize MPU6050
  mpu.setTempSensorEnabled(false);
  mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setSleepEnabled(false);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful")
                                      : F("MPU6050 connection failed"));
  servoPID.SetSampleTime(15);
  servoPID.SetOutputLimits(0, 180);

  // mpu.CalibrateGyro();
  // mpu.CalibrateAccel();
  mpu.setXAccelOffset(-2764);
  mpu.setYAccelOffset(2209);
  mpu.setZAccelOffset(2674);
  mpu.setXGyroOffsetTC(-122);
  mpu.setYGyroOffsetTC(31);
  mpu.setZGyroOffsetTC(-3);
  mpu.PrintActiveOffsets();

  // Servo Setup
  servo.attach(SERVO_PIN);
  servo.write(servoPos);

  // Fan Setup
  pinMode(LEVITATION_FAN_PIN, OUTPUT);
  pinMode(THRUST_FAN_PIN, OUTPUT);

  // IR Sensor Setup
  setupADC();

  // Initialize Timers
  prevTime = millis();

  setLevitationSpeed(LEVITATION_MAX_SPEED);
  setThrustSpeed(THRUST_MAX_SPEED);
}

void loop() {
  int16_t gyroX, gyroY, gyroZ;
  int16_t accelX, accelY, accelZ;

  uint32_t curTime = millis();
  uint32_t dt = curTime - prevTime;

  if (dt > 15) {
    mpu.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
    rawCurAngle += dt * gyroZ;
    curAngle = rawCurAngle / sensFacGyro;

    rawVelX += dt * accelX;
    rawVelY += dt * accelY;
    if (abs(targetAngle - curAngle) < 15) {
      setThrustSpeed(THRUST_MAX_SPEED);
    } else {
      setThrustSpeed(210);
    }

    curSpeed = sqrt(rawVelX * rawVelX + rawVelY * rawVelY) / sensFacAccel;

    // Update servo motor
    double serveA = 90.0 - (targetAngle - curAngle);
    double angSpeed = (double)gyroZ / sensFacGyro;
    if (angSpeed < 20) {
      // serveA = constrain(serveA, 0, 180);
    } else {
    }
    serveA = constrain(serveA, SERVO_MIN_MAX, 180 - SERVO_MIN_MAX);
    servo.write(serveA);
    // servo.write(90);

    prevTime = curTime;
  }

  if (usTimer.repeat()) {
    distFront = frontSensor.measureDistanceCm();
  }

  if (irTimer.repeat()) {
    // Convert analog value to distance (formula from datasheet)
    distWall = 27 * pow((readADCH() / 256.0) * V_REF, -1.173);
  }

#if DEBUG
  if (logTimer.repeat()) {
    setLevitationSpeed(LEVITATION_MAX_SPEED);
    // Serial.println("Print LINE");
    // Serial.println(rawCurAngle);

    // Serial.print("Raw Angle: ");
    // Serial.println(gyroZ);
    // Serial.print("Cur Angle: ");
    // Serial.println(curAngle);
    // Serial.print("Tar Angle: ");
    // Serial.println(targetAngle);
    // Serial.print("Dist Front: ");
    // Serial.println(distFront);
    Serial.print("Dist Wall: ");
    Serial.println(distWall);

    // Serial.println("Accel:");
    // Serial.println(accelX);
    // Serial.println(accelY);
    // Serial.println(accelZ);

    // double pVelX = rawVelX / sensFacAccel;
    // double pVelY = rawVelY / sensFacAccel;
    // double pVelZ = rawVelZ / sensFacAccel;
    // double speed = sqrt(pVelX * pVelX + pVelY * pVelY + pVelZ * pVelZ);
    Serial.print("Velocity: ");
    Serial.println(curSpeed);
    Serial.println(rawVelX / sensFacAccel);
    Serial.println(rawVelY / sensFacAccel);
    // Serial.println(pVelX);
    // Serial.println(pVelY);
    // Serial.println(pVelZ);
    Serial.println("");
  }
#endif

  machine.run();
}

void lockTargetAngle(double angle) { targetAngle += angle; }

void StraightPathState() {
  // if (!preSlowTimer.started() && !preSlowTimer.done() && distFront < 100 &&
  //     abs(targetAngle - curAngle) < ANGLE_TURN_THRESH) {
  //   setLevitationSpeed(100);
  // } else if (preSlowTimer.done()) {
  //   setLevitationSpeed(LEVITATION_MAX_SPEED);
  // }
#if DEBUG
  if (logStateTimer.repeat()) {
    Serial.println("State: Straight");
  }
#endif
}

boolean CheckIfNeeds90Correct() {
  // Check if the angle stabilizes
  if ((abs(targetAngle + 90 - curAngle) < ANGLE_TURN_THRESH ||
       abs(targetAngle - 90 - curAngle) < ANGLE_TURN_THRESH)) {
    if (!correctTimer.started()) {
      Serial.println("Started Correction Timer.");
      correctTimer.start();
    } else if (correctTimer.done()) {
      // Do corrective measures if the angle is stabilized
      if (abs(targetAngle + 90 - curAngle) < ANGLE_TURN_THRESH) {
        double deltaAngle = (CORRECT_DIST_FRONT < distFront) ? 90.0 : 270.0;
        lockTargetAngle(deltaAngle);
      } else if (abs(targetAngle - 90 - curAngle) < ANGLE_TURN_THRESH) {
        double deltaAngle = (CORRECT_DIST_FRONT < distFront) ? -90.0 : -270.0;
        lockTargetAngle(deltaAngle);
      }
      Serial.println("Correcting mistake.");

      preSlowTimer.reset();
      correctTimer.reset();
      turnTimer.start();
      return true;
    }
  } else if (correctTimer.started()) {
    Serial.println("Resetting Correction Timer.");
    correctTimer.reset();
  }

  return false;
}

void Corrective90Wall() {
  // if (!correctThrustTimer.started()) {
  //   correctThrustTimer.start();
  //   setThrustSpeed(0);
  // } else if (correctThrustTimer.done()) {
  //   setThrustSpeed(THRUST_MAX_SPEED);
  // }

#if DEBUG
  if (logStateTimer.repeat()) {
    Serial.println("State: Corrective");
  }
#endif
}

boolean CheckIfOnTurn() {
  if (TURN_TRIGGER_DIST > distFront && distFront > 0 &&
      abs(targetAngle - curAngle) < ANGLE_TURN_THRESH) {
    Serial.println("TRIGGER");
    double deltaAngle = (TURN_SIDE_DIST > distWall) ? 90.0 : -90.0;
    lockTargetAngle(deltaAngle);
    turnTimer.start();
    preSlowTimer.reset();
    return true;
  }

  return false;
}

void TurnState() {
#if DEBUG
  if (logStateTimer.repeat()) {
    Serial.println("State: Turn");
  }
#endif
}

boolean CheckIfOnStraight() {
  // // Slow down for x ms.
  if (!slowdownTimer.started()) {
    slowdownTimer.start();
    setLevitationSpeed(0);
  } else if (slowdownTimer.done()) {
    setLevitationSpeed(LEVITATION_MAX_SPEED);
  }

  if (abs(targetAngle - curAngle) < ANGLE_TURN_THRESH && turnTimer.done()) {
    correctThrustTimer.reset();
    turnTimer.reset();
    slowdownTimer.reset();
    preSlowTimer.reset();
    Serial.println("TRIGGER");
    return true;
  }
  return false;
}
