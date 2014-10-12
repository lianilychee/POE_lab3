#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
 
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
const int ir_read = A0;
int state;
int counter=0;
int sensorVal;
int desired_pos_slice;
int error;
int P;
int desired_pos_deg = 180; // desired position in degrees; SET THIS FOR TESTING
int kp = 2;
int slice_angle = 15;
int error_threshold = 20;
 
void setup() {
  Serial.begin(9600);  // set up Serial library at 9600 bps
  AFMS.begin();        // create with the default frequency 1.6KHz
  myMotor->setSpeed(50);
  myMotor->run(FORWARD);
}
 
void loop() {
  uint8_t i;
  myMotor->run(FORWARD);
 
  // // Convert desired_pos to ticks
  desired_pos_slice = (desired_pos_deg / slice_angle) + 1; // denominator is angle of each slice; SET THIS FOR NEW PINWHEEL
  detect_slice();
  adjust_speed();
}
 
void adjust_speed(){
  // error = how much distance is left
  error = desired_pos_deg - (counter * slice_angle);
  P = error * kp;
  if (P > 200){ P=200; }
  if (error > error_threshold){ myMotor->setSpeed(P); }
  else if (error < 0){ myMotor->run(BACKWARD); myMotor->setSpeed(P); }
  else{ myMotor->setSpeed(0); }
}
 
 
void detect_slice(){
  sensorVal = analogRead(ir_read);
  // Detect slice change and increment counter;
  // if sensing black slice when prev state is white,
  if (sensorVal > 900 && state == 1) { state = 0; counter++; }
  // if sensing white slice when prev state is black,
  if (sensorVal < 790 && state == 0) { state = 1; counter++; }
 
  Serial.print("state: "); Serial.println(state);
  Serial.print("ir_read: "); Serial.println(sensorVal);
  Serial.print("COUNT: "); Serial.println(counter);
  delay(50);
}
