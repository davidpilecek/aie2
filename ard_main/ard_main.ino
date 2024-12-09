#include "SoftPWM.h"

//front right wheel
const byte FRA = 5; //B1A right
const byte FRB = 2; //B2A(B1B) right       
//rear right wheel
const byte RRA = 11; //A1B right
const byte RRB = 12; //A1A right    
//rear left wheel
const byte RLA = 10; // B1A left
const byte RLB = 9; // B2A(B1B) left      
//front left wheel
const byte FLA = 6; //A1B left
const byte FLB = 3; //A1A left

const byte motorPins[] = {FRA, FLA, RRA, RLA, FRB, FLB, RRB, RLB};

int numbers[8];

void setup() {
  
  for(byte i = 0;i<sizeof(motorPins);i++){pinMode(motorPins[i], OUTPUT);};

  Serial.begin(9600);
  SoftPWMBegin();

}
void loop() {

  if (Serial.available() >= 8) {
    
  for (int i = 0; i < 8; i++) {
            numbers[i] = Serial.read();
             
            Serial.write(numbers[i]);     
  }
  
  for (int j = 0; j < 8; j++) {
            
            SoftPWMSetPercent(motorPins[j], numbers[j]);
            
  }
  
  }
  
}
