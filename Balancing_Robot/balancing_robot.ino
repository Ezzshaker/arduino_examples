/* Self Balancing Robot (ver 1.0) - FINAL PROJECT
 * Eric Ouyang - 11/30/2012
 * COMP-600: Independent Project
 * Advisor: Ms. Litvin
 */

/* LIBRARIES */
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Servo.h>

/* PIN DEFINITIONS */
// LEDS
static int greenLED = 4;
static int redLED = 5;

// Motor
static int motorPin = 8;

// Potentiometers
static int pot1Pin = A0;
static int pot2Pin = A1;
static int pot3Pin = A3;

/* CONSTANT PARAMETERS */

// from MPU datasheet
const float accScale = 8192;
const float gyroScale = 131;

// number of samples for the calibration
const int numGyroSamples = 100;

const float gyroWeight = .90;
const float accWeight = .10;
// where weight constants add up to 1

// motor vars
const int neutralPulseLength = 1500; // in microseconds

// PID calc. consts
float Kp = 15; // proportional term 
float Ki = 0;  // integral term
float Kd = 0;  // derivative term - can't get it to remotely balance with Kp or Ki

const int numAnglesToAvg = 3;

const float targetAngle = -1;

/* VARIABLES */
MPU6050 mpu;
int16_t accX, accY, accZ; // container vars for accelerometer readings
float gyroOffset = 0; // set in calibrateGyro()
float currAngle, prevAngle;
float prevAngles[numAnglesToAvg];
int prevAnglesIndex = 0;

Servo motor;
int motorSpeed;
int pulseLength;

// time vars
int currTime = 0; // microseconds
int prevTime = 0; // microseconds
float deltaTime;  // seconds

// for PID calcs
float errorSum = 0;
float currError = 0;
float prevError = 0;
float iTerm;
float dTerm;

// initialize and configure
void setup() {
  // LEDS
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  digitalWrite(redLED, HIGH); // setting up, turn red LED on
  
  // Motor
  motor.attach(motorPin);
  
  // Serial
  Serial.begin(115200);

  // Join I2C bus
  Wire.begin(); 

  // Initializing MPU
  mpu.initialize();

  // Testing MPU
  Serial.println(mpu.testConnection() ? "MPU is good!" : "Oops! Check the connections");

  calibrateGyro();
  Serial.print("gyroOffset: ");
  Serial.println(gyroOffset);

  // All set! Turn green LED on, red LED off
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, HIGH);
}

// main loop
void loop() {
  // For Processing readout
  currTime = micros();
  deltaTime = (currTime - prevTime) / 1000000.0;
  prevTime = currTime;
  
  updateAngle();
  Serial.print("$,");
  Serial.print(accAngle());
  Serial.print(",");
  Serial.print(currAngle);
  Serial.print(",%");
  
  /*
  currTime = micros();
  deltaTime = (currTime - prevTime) / 1000000.0;
  Serial.print("deltaTime:\t");
  Serial.print(deltaTime, 5);
  prevTime = currTime;
  
  updateAngle();
  Serial.print("\tcurrAngle:\t");
  Serial.print(currAngle);
  */
  
  Kp = map(analogRead(pot1Pin), 0, 1024, 0, 50);
  Ki = map(analogRead(pot2Pin), 0, 1024, 0, 20);
  Kd = map(analogRead(pot3Pin), 0, 1024, 0, 50);
  
  /*
  Serial.print("\tKp:\t");
  Serial.print(Kp);
  Serial.print("\tKi:\t");
  Serial.print(Ki);
  Serial.print("\tKd:\t");
  Serial.print(Kd);
  */
  
  updateSpeed();
  /*
  Serial.print("\tcurrError:\t");
  Serial.print(currError);
  Serial.print("\terrorSum:\t");
  Serial.print(errorSum);
  Serial.print("\tmotorSpeed:\t");
  Serial.print(motorSpeed);
  Serial.print("\tiTerm:\t");
  Serial.print(iTerm);
  Serial.print("\tdTerm:\t");
  Serial.print(dTerm);
  */
  
  updateMotor();
  //Serial.print("\tpulseLength:\t");
  //Serial.println(pulseLength);
}

void updateAngle() {
  prevAngles[prevAnglesIndex] = gyroWeight * (prevAngle + ((float) mpu.getRotationZ() / gyroScale - gyroOffset) * deltaTime) + accWeight * accAngle();
  prevAnglesIndex = (prevAnglesIndex + 1) % numAnglesToAvg;
  
  float sum = 0;
  for (int i = 0; i < numAnglesToAvg; i++)
      sum += prevAngles[i];
      
  currAngle = sum / numAnglesToAvg;
  prevAngle = currAngle;
}

void updateSpeed() {
  /* First Algorithm - basic proportional function - DOES NOT WORK
  
  if (currAngle > 0 && currAngle < 30)
    motorSpeed = map(currAngle, 0, 30, 1300, 1000);
  else if (currAngle < 0 && currAngle > -30)
    motorSpeed = map(currAngle, -30, 0, 2000, 1700);
  else
    motorSpeed = 1500;

  motor.writeMicroseconds(motorSpeed);
  
  */
  
  // PID implementation
  
  currError = targetAngle - currAngle;
  
  // Proportional Term: pTerm = currAngle;
  
  // Integral Term
  errorSum += currError;
  errorSum = constrain(errorSum, -50, 50);
  iTerm = Ki * errorSum ;
  
  // Derivative Term
  dTerm = Kd * (currError - prevError);
  
  motorSpeed = Kp * currError + iTerm + dTerm;
  
  if (currError > 30 || currError < -30)
    motorSpeed = 0;
    
  prevError = currError;
}

void updateMotor() {
  pulseLength = constrain(neutralPulseLength - motorSpeed, 1000, 2000);
  motor.writeMicroseconds(pulseLength);
}

// angle calculated from accelerometer
float accAngle() {
  mpu.getAcceleration(&accX, &accY, &accZ);

  float radius = sqrt((float)accX * (float)accX + (float)accY * (float)accY);
  return (float) accX / radius / PI * 180; // sin approximation
}

void calibrateGyro() {
  float sum = 0;
  for (int i = 0; i < numGyroSamples; i++)
  {
    sum += (float) mpu.getRotationZ() / gyroScale;
    delay(10);
  }

  gyroOffset = sum / numGyroSamples;
}
