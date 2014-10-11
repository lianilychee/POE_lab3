
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
const int ir_read = A0;

void setup() {
  Serial.begin(9600);  // set up Serial library at 9600 bps
  AFMS.begin();        // create with the default frequency 1.6KHz
  
  myMotor->setSpeed(100);
  myMotor->run(FORWARD);
  myMotor->run(RELEASE);
  
  int scope = 0;
}

void loop() {
  Serial.println(analogRead(ir_read));
  delay(3);
  uint8_t i;
  myMotor->run(FORWARD);
}

void detect_pos(){
  
  int state = 0;
  int sensorValue = 0;
  int counter;
  int angle;
  
  if (state == 0 && sensorValue<960){
    state = 1;
    counter++;
    angle += counter*30;
 
  }
  if (state == 1 && sensorValue>1000){
    state = 0;
    counter++;
    angle += counter*30;
  }
 
 Serial.print("counter: ");
 Serial.println(counter);
  
}
