
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
const int ir_read = A0;
int sensorValue = 0;
int counter;
int state;


void setup() {
  Serial.begin(9600);  // set up Serial library at 9600 bps
  AFMS.begin();        // create with the default frequency 1.6KHz
  
  myMotor->setSpeed(75);
  myMotor->run(FORWARD);
  myMotor->run(RELEASE);
  
  int scope = 0;
}

void loop() {
//  Serial.println(analogRead(ir_read));
  delay(3);
  uint8_t i;
  myMotor->run(FORWARD);

  if (sensorValue < 1000) { // if sensing black slice
    state = 0;
    counter++;
  }
  if (sensorValue > 1000) { // if sensing white slice
    state = 1;
    counter++;
  }
  
  Serial.print("counter: "); Serial.println(counter);
  Serial.println(state);
  delay(3);
  
//  detect_pos();

}

void detect_pos(){
  
  int state = 0;
  int sensorValue = 0;
  int counter;
  int angle;
  
  // Threshold value must be changed according to ambient lighting
  if (state == 0 && sensorValue<1000){ // If state is black and sensorVal < 1000
    state = 1;
    counter = counter + 1;
    angle += counter*30;
    Serial.print("counter: ");
    Serial.println(counter);
  }
  if (state == 1 && sensorValue>1000){ // If state is white and sensorVal > 1000
    state = 0;
    counter = counter + 1;
    angle += counter*30;
    Serial.print("counter: ");
    Serial.println(counter);
  }

  Serial.print("counter: ");
  Serial.println(counter);
  
}
