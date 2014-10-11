// Basic reading of the IR sensor

// Pin definition
const int ir_read = A0;  // Infrared sensor to Analog 0

void setup() {
  Serial.begin(9600); 
}

void loop() {
  
  Serial.println(analogRead(ir_read));
  delay(3);
            
}
