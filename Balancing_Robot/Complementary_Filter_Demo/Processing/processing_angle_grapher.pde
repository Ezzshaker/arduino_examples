// graphing the angle readouts 

import processing.serial.*;

Serial arduino;
int xPos = 1;

String[] packet; 

float accelAngle;
float gyroAngle;
float realAngle;

float mappedAccelAngle;
float mappedGyroAngle;
float mappedRealAngle;

void setup() {
  size(1000, 200);
  background(0);
  
  println(Serial.list()); 
  arduino = new Serial(this, Serial.list()[0], 115200);
}

void draw() {
  float mappedAccelAngle = map(accelAngle, -30, 30, 0, height);
  float mappedGyroAngle = map(gyroAngle, -30, 30, 0, height);
  float mappedRealAngle = map(realAngle, -30, 30, 0, height);
  
  stroke(255, 0, 0);
  ellipse(xPos, mappedAccelAngle, 2, 2);
  
  stroke(0, 255, 0);
  ellipse(xPos, mappedGyroAngle, 2, 2);
  
  stroke(0, 0, 255);
  ellipse(xPos, mappedRealAngle, 2, 2);
  
  if (xPos < width)
  {
    xPos++; 
  }
  else
  {
    xPos = 0;
    background(0);
  }
  
}

void serialEvent(Serial arduino) {
    String input = arduino.readStringUntil('%');
    if (input != null) {
      println(input);
      packet = splitTokens(input, ",");
      if (packet[0].equals("$") && packet[4].equals("%")){
        accelAngle = Float.parseFloat(packet[1]);
        gyroAngle = Float.parseFloat(packet[2]);
        realAngle = Float.parseFloat(packet[3]);
      }
    }
}
