//front right wheel
const byte FRA = 5; //B1A right
const byte FRB = 2; //B2A right
//rear right wheel
const byte RRA = 9; //A1B right
const byte RRB = 11; //A1A right
//rear left wheel
const byte RLA = 10; // B1A left
const byte RLB = 12; // B2A left
//front left wheel
const byte FLA = 6; //A1B left
const byte FLB = 3; //A1A left

byte SPEED = 100;

const byte motorPins[] = {FRA, FLA, RRA, RLA, FRB, FLB, RRB, RLB};

void spin(byte speed, byte mot_num){

analogWrite(motorPins[mot_num], speed);
analogWrite(motorPins[mot_num + 4], 0);

}

void spin_all(byte speed){

analogWrite(motorPins[0], speed);
analogWrite(motorPins[4], 0);
analogWrite(motorPins[1], speed);
analogWrite(motorPins[5], 0);
analogWrite(motorPins[2], speed);
analogWrite(motorPins[6], 0);
analogWrite(motorPins[3], speed);
analogWrite(motorPins[7], 0);

}

void stop_all(){

for(byte i = 0;i<sizeof(motorPins);i++){

analogWrite(motorPins[i], 0);

}

};

int numbers[4];

void setup() {
for(byte i = 0;i<sizeof(motorPins);i++){pinMode(motorPins[i], OUTPUT);};
  
  Serial.begin(9600);
/*
spin(100, 1);
delay(500);
stop_all();
*/
}
void loop() {
 
  if (Serial.available() >= 4) {
    
  for (int i = 0; i < 4; i++) {
            numbers[i] = Serial.read();      
  }
  
  Serial.write(numbers[0]);
  for (int j = 0; j < 4; j++) {
            
            spin(numbers[j], j);
  }
  
   
  }
  
}
