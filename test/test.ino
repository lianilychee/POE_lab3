
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
 
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
int sensorPin = A0;    // select the input pin for the potentiometer
//int sensorValue = 0; UNCOMMENT WHEN SENSOR HOOKED UP
 
void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  AFMS.begin();  // create with the default frequency 1.6KHz; speed 0 (off) to 255 (max)
  myMotor->setSpeed(200);
  myMotor->run(FORWARD);
  myMotor->run(RELEASE);
}
 
void loop() {
  uint8_t i; // no one knows what this is...
  
//  desired_pos = 180; // THIS IS WHAT HAS TO CHANGE; angle units in degrees
  
 
//  sensorValue_prev = sensorValue_current
//  sensorValue = analogRead(sensorPin); UNCOMMENT WHEN SENSOR HOOKED UP
//  delay(10); UNCOMMENT WHEN SENSOR HOOKED UP
//  sensorValue_now = sensorValue;
//  Serial.println(sensorValue); UNCOMMENT WHEN SENSOR HOOKED UP
 
  myMotor->run(FORWARD);
  myMotor->setSpeed(100);
  
}

//void detect_pos() {
//  once a slice is detected, 
//  subtract the angle of each slice from our intended pos
//}
//
//void calc_angle(desired_pos) {
//  Serial.println("hello")
//}
