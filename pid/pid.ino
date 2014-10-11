
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
const int ir_read = A0;
int state;
int counter;

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

//  detect_slice();

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
  
  if (counter == 9) {
    Serial.println("RELEASE THE KRAKEN");
    myMotor->setSpeed(0);
  }
  
}

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
