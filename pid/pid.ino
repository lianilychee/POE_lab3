
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
const int ir_read = A0;
int state;
int counter;
int desired_pos_deg = 90; // desired position in degrees; SET THIS FOR TESTING

void setup() {
  Serial.begin(9600);  // set up Serial library at 9600 bps
  AFMS.begin();        // create with the default frequency 1.6KHz
  
  myMotor->setSpeed(75);
  myMotor->run(FORWARD);
  myMotor->run(RELEASE);
}

void loop() {
  
  uint8_t i;
  myMotor->run(FORWARD);
  
  
  // // Convert desired_pos to ticks
  int desired_pos_slice = desired_pos_deg / 36; // denominator is angle of each slice; SET THIS FOR NEW PINWHEEL
  
  // // Detect slice change and increment counter;
  if (analogRead(ir_read) < 1000 && state == 1) { // if sensing black slice when prev state is white,
    state = 0;
    counter++;
  }
  if (analogRead(ir_read) >= 1000 && state == 0) { // if sensing white slice when prev state is black,
    state = 1;
    counter++;
  }
  Serial.print("state: "); Serial.println(state);
  Serial.print("COUNT: "); Serial.println(counter);
  delay(300);
  
  // // If desired position is reached, stop the motor
  if (counter == desired_pos_slice) {
    myMotor->setSpeed(0);
  }
  
  
  
  
}





// detect_slice() may or may not be necessary
void detect_slice(){
  
  int state = 0;
  int sensorValue = 0;
  int counter;
  int angle;
  
  Serial.println(counter);
  
  if (analogRead(ir_read) < 1000 && state == 1) { // if sensing black slice when prev state is white,
    state = 0;
//    counter++;
  }
  if (analogRead(ir_read) >= 1000 && state == 0) { // if sensing white slice when prev state is black,
    state = 1;
//    counter++;
  }
  
  Serial.print("state: "); Serial.println(state);
  Serial.print("COUNT: "); Serial.println(counter);
  
}
