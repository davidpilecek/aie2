//front right wheel
const byte FRA = 5; //BIA
const byte FRB = 2; //BIB
//rear right wheel
const byte RRA = 11; //AIA right
const byte RRB = 9; //AIB right
//rear left wheel
const byte RLA = 12; // B2A left
const byte RLB = 10; // BIA left
//front left wheel
const byte FLA = 3; //AIA left
const byte FLB = 6; //AIB left

byte speed = 100;

const byte motorPins[] = {FRA, FRB, FLB, FLA, RRA, RRB, RLB, RLA};


void spin(byte speed, byte index){

analogWrite(motorPins[index], speed);
analogWrite(motorPins[index + 1], 0);

}

void spin_all(byte speed){

analogWrite(motorPins[0], speed);
analogWrite(motorPins[1], 0);
analogWrite(motorPins[2], speed);
analogWrite(motorPins[3], 0);
analogWrite(motorPins[4], speed);
analogWrite(motorPins[5], 0);
analogWrite(motorPins[6], speed);
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
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 0);
  
}

void loop() {
 
  if (Serial.available() >= 4) {
    
    
  for (int i = 0; i < 4; i++) {
            numbers[i] = Serial.read();
        }

    
    switch(numbers[0]){

      case 0:
        Serial.write(numbers[0]);
        stop_all();
        break;
      case 1:
        Serial.write(numbers[0]);
        spin_all(speed);
        break;
      default:
      break;
      }   
     
     switch(numbers[1]){

      case 10:
        Serial.write(numbers[1]);
        stop_all();
        break;
      case 11:
        Serial.write(numbers[1]);
        spin_all(speed);
        break;
      default:
      break;
      }   
    switch(numbers[2]){

      case 20:
        Serial.write(numbers[2]);
        stop_all();
        break;
      case 21:
        Serial.write(numbers[2]);
        spin_all(speed);
        break;
      default:
      break;
      }  
      switch(numbers[3]){

      case 30:
        Serial.write(numbers[3]);
        stop_all();
        break;
      case 31:
        Serial.write(numbers[3]);
        spin_all(speed);
        break;
      default:
      break;
      }  
  }
  
}
