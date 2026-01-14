/*******************************************************************************************
 * PROJECT: Autonomous USV Navigation & Fire-Rescue System
 * AUTHOR : Hatem Safwat Mohamed
 * SYSTEM : GPS + IMU + Kalman Filter + LOS Guidance + PID Control
 * PLATFORM: Arduino Mega / Due (Recommended)
 *
 * DESCRIPTION:
 * This file implements a complete autonomous guidance, navigation,
 * and control stack for an Unmanned Surface Vehicle (USV).
 *
 * MAIN MODULES:
 * 1. Hardware Initialization
 * 2. GPS Processing
 * 3. IMU Processing
 * 4. Kalman Sensor Fusion
 * 5. Coordinate Transformation
 * 6. Obstacle Detection
 * 7. LOS Guidance Law
 * 8. PID Heading Control
 * 9. Motor Actuation
 * 10. Diagnostics & Safety
 *******************************************************************************************/

/******************************** LIBRARIES ********************************/
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>
#include <PID_v1.h>
#include <math.h>

/******************************** PIN CONFIG ********************************/
// Motor Driver
#define ENA 7
#define ENB 2
#define IN1 6
#define IN2 5
#define IN3 4
#define IN4 3

// GPS
#define RX_GPS 17
#define TX_GPS 16

// Ultrasonic Sensor
#define US_TRIG 1
#define US_ECHO 0

// Indicators
#define LED_STATUS 13

/******************************** SERIAL ********************************/
SoftwareSerial GpsSerial(RX_GPS, TX_GPS);
TinyGPSPlus gps;

/******************************** IMU ********************************/
Adafruit_MPU6050 mpu;

/******************************** GLOBAL CONSTANTS ********************************/
const float EARTH_RADIUS = 6.3781e6;
const float DEG2RAD = PI / 180.0;
const float RAD2DEG = 180.0 / PI;

/******************************** REFERENCE ORIGIN ********************************/
float originLat = 27.18799450;
float originLng = 31.17490267;

/******************************** TARGET WAYPOINT ********************************/
float originalTargetLat = 27.18816046;
float originalTargetLng = 31.17505422;

float activeTargetLat;
float activeTargetLng;

/******************************** STATE MACHINE ********************************/
enum USVState {
  NAVIGATION,
  OBSTACLE_AVOIDANCE,
  TARGET_REACHED,
  EMERGENCY_STOP
};

USVState currentState = NAVIGATION;

/******************************** GPS VARIABLES ********************************/
double gpsLat, gpsLng;
float gpsSpeed;
unsigned long gpsTime;

/******************************** IMU VARIABLES ********************************/
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

float accAngleX, accAngleY;
float gyroAngleX = 0, gyroAngleY = 0;
float fusedAngleX, fusedAngleY;

/******************************** KALMAN FILTERS ********************************/
SimpleKalmanFilter kalmanAccelX(0.2, 0.4, 0.01);
SimpleKalmanFilter kalmanAccelY(0.2, 0.4, 0.01);
SimpleKalmanFilter kalmanAngleX(0.2, 0.4, 0.01);
SimpleKalmanFilter kalmanLat(0.2, 0.4, 0.01);
SimpleKalmanFilter kalmanLng(0.2, 0.4, 0.01);

/******************************** FILTERED VALUES ********************************/
float kalmanLatVal, kalmanLngVal;
float kalmanAngleVal;

/******************************** POSITION ********************************/
float currentX, currentY;
float targetX, targetY;

/******************************** PID ********************************/
double Input, Output, Setpoint;
double Kp = 2.0, Ki = 2.0, Kd = 0.2;
PID headingPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

/******************************** MOTOR ********************************/
const int MAX_PWM = 255;

/******************************** TIME ********************************/
unsigned long prevTime = 0;

/******************************** OBSTACLE ********************************/
long usDuration;
float usDistance;
bool obstacleDetected = false;

/******************************** FUNCTIONS ********************************/

float convertGPSToX(float lon, float lat) {
  return (lon - originLng) * DEG2RAD * EARTH_RADIUS * cos(lat * DEG2RAD);
}

float convertGPSToY(float lon, float lat) {
  return (lat - originLat) * DEG2RAD * EARTH_RADIUS;
}

float calculateLOS(float currX, float currY, float targX, float targY) {
  return atan2(targX - currX, targY - currY) * RAD2DEG;
}

/******************************** MOTOR CONTROL ********************************/
void applyMotorControl(double correction) {
  int leftPWM = constrain(MAX_PWM + correction, 0, MAX_PWM);
  int rightPWM = constrain(MAX_PWM - correction, 0, MAX_PWM);

  analogWrite(ENA, leftPWM);
  analogWrite(ENB, rightPWM);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

/******************************** OBSTACLE DETECTION ********************************/
bool checkObstacle() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);

  usDuration = pulseIn(US_ECHO, HIGH, 30000);
  usDistance = usDuration * 0.034 / 2.0;

  return (usDistance > 0 && usDistance < 20);
}

/******************************** GPS + IMU UPDATE ********************************/
void updateSensors() {

  // GPS
  while (GpsSerial.available()) {
    gps.encode(GpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    gpsLat = gps.location.lat();
    gpsLng = gps.location.lng();
    gpsSpeed = gps.speed.mps();
    gpsTime = millis();
  }

  // IMU
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;

  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;

  accAngleX = atan(gyroY / sqrt(gyroX * gyroX + gyroZ * gyroZ)) * RAD2DEG;

  float dt = (millis() - prevTime) / 1000.0;
  prevTime = millis();

  gyroAngleX += gyroX * dt;

  fusedAngleX = 0.98 * gyroAngleX + 0.02 * accAngleX;

  kalmanAngleVal = kalmanAngleX.updateEstimate(fusedAngleX);
  kalmanLatVal = kalmanLat.updateEstimate(gpsLat);
  kalmanLngVal = kalmanLng.updateEstimate(gpsLng);
}

/******************************** SETUP ********************************/
void setup() {
  Serial.begin(9600);
  GpsSerial.begin(9600);
  Wire.begin();

  pinMode(LED_STATUS, OUTPUT);
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  mpu.begin();

  headingPID.SetMode(AUTOMATIC);
  headingPID.SetOutputLimits(-120, 120);

  activeTargetLat = originalTargetLat;
  activeTargetLng = originalTargetLng;

  prevTime = millis();
}

/******************************** LOOP ********************************/
void loop() {

  updateSensors();

  currentX = convertGPSToX(kalmanLngVal, kalmanLatVal);
  currentY = convertGPSToY(kalmanLngVal, kalmanLatVal);

  obstacleDetected = checkObstacle();

  switch (currentState) {

    case NAVIGATION:
      if (obstacleDetected) {
        currentState = OBSTACLE_AVOIDANCE;
      }
      break;

    case OBSTACLE_AVOIDANCE:
      activeTargetX = currentX - 1.5;
      activeTargetY = currentY + 1.5;
      if (abs(activeTargetX - targetX) < 0.3) {
        currentState = NAVIGATION;
      }
      break;

    case TARGET_REACHED:
      stopMotors();
      digitalWrite(LED_STATUS, HIGH);
      return;

    case EMERGENCY_STOP:
      stopMotors();
      return;
  }

  targetX = convertGPSToX(activeTargetLng, activeTargetLat);
  targetY = convertGPSToY(activeTargetLng, activeTargetLat);

  Setpoint = calculateLOS(currentX, currentY, targetX, targetY);
  Input = kalmanAngleVal;

  headingPID.Compute();
  applyMotorControl(Output);

  Serial.print("Lat: "); Serial.print(kalmanLatVal, 6);
  Serial.print(" Lng: "); Serial.print(kalmanLngVal, 6);
  Serial.print(" Heading: "); Serial.print(kalmanAngleVal);
  Serial.print(" PID: "); Serial.println(Output);
}

