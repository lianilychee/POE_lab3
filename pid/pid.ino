
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
const int ir_read = A0;
int counter;
int state;


void setup() {
  Serial.begin(9600);  // set up Serial library at 9600 bps
  AFMS.begin();        // create with the default frequency 1.6KHz
  
  myMotor->setSpeed(50);
  myMotor->run(FORWARD);
  myMotor->run(RELEASE);
}

void loop() {
  uint8_t i;
  myMotor->run(FORWARD);

//  Serial.print("sensorValue: "); Serial.println(analogRead(ir_read));
  Serial.println(analogRead(ir_read));
  if (analogRead(ir_read) < 1000 && state == 1) { // if sensing black slice when prev state is white,
    state = 0;
    counter++;
  }
  if (analogRead(ir_read) >= 1000 && state == 0) { // if sensing white slice when prev state is black,
    state = 1;
    counter++;
  }
  
  Serial.print("state: "); Serial.println(state);
  Serial.println(counter);
  
  delay(3);
  
  detect_slice();

}

void detect_slice(){
  
  int state = 0;
  int sensorValue = 0;
  int counter;
  int angle;
  
  if (analogRead(ir_read) < 1000 && state == 1) { // if sensing black slice when prev state is white,
    state = 0;
    counter++;
  }
  if (analogRead(ir_read) >= 1000 && state == 0) { // if sensing white slice when prev state is black,
    state = 1;
    counter++;
  }
  
  Serial.print("state: "); Serial.println(state);
  Serial.println(counter);
  
}
