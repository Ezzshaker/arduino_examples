/* Filtered Angle Packet - FINAL PROJECT
 * Eric Ouyang - 11/20/2012
 * COMP-600: Independent Project
 * Advisor: Ms. Litvin
 */

/* LIBRARIES */ 
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

/* PIN DEFINITIONS */
// LEDS
static int greenLED = 4;
static int redLED = 5;

/* CONSTANT PARAMETERS */

// from MPU datasheet
const float accScale = 8192;
const float gyroScale = 131;

// number of samples for the calibration
const int numGyroSamples = 100;

const float gyroWeight = .90;
const float accWeight = .10;
// where weight constants add up to 1

const int numAnglesToAvg = 3;

/* VARIABLES */
MPU6050 mpu;
int16_t accX, accY, accZ; // container vars for accelerometer readings
float gyroOffset = 0; // set in calibrateGyro()
float currAngle, prevAngle;
float prevAngles[numAnglesToAvg];
float gyroAngle = 0;
int prevAnglesIndex = 0;

// time vars
int currTime = 0; // microseconds
int prevTime = 0; // microseconds
float deltaTime;  // seconds

// initialize and configure
void setup() {
  // LEDS
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  digitalWrite(redLED, HIGH); // setting up, turn red LED on
  
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
  // For Processing angle readout
  currTime = micros();
  deltaTime = (currTime - prevTime) / 1000000.0;
  prevTime = currTime;
  
  updateAngle();
  Serial.print("$,");
  Serial.print(accAngle());
  Serial.print(",");
  Serial.print(gyroAngle);
  Serial.print(",");
  Serial.print(currAngle);
  Serial.print(",%");
}

void updateAngle() {
  prevAngles[prevAnglesIndex] = gyroWeight * (prevAngle + ((float) mpu.getRotationZ() / gyroScale - gyroOffset) * deltaTime) + accWeight * accAngle();
  prevAnglesIndex = (prevAnglesIndex + 1) % numAnglesToAvg;
  
  float sum = 0;
  for (int i = 0; i < numAnglesToAvg; i++)
      sum += prevAngles[i];
      
  currAngle = sum / numAnglesToAvg;
  prevAngle = currAngle;
  
  // not related to current angle calc... used in angle grapher
  gyroAngle += ((float) mpu.getRotationZ() / gyroScale - gyroOffset) * deltaTime;
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
