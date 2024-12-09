#include "SoftPWM.h"


//connect only FL, RL to PWM

//front right wheel
const byte FRA = 5; //B1A right
const byte FRB = 2; //B2A(B1B) right       !NO PWM!
//rear right wheel
const byte RRA = 11; //A1B right
const byte RRB = 12; //A1A right           !NO PWM!
//rear left wheel
const byte RLA = 10; // B1A left
const byte RLB = 9; // B2A(B1B) left      
//front left wheel
const byte FLA = 6; //A1B left
const byte FLB = 3; //A1A left


const byte motorPins[] = {FRA, FLA, RRA, RLA, FRB, FLB, RRB, RLB};


void spin(byte speed_drive, byte mot_num){

SoftPWMSetPercent(motorPins[mot_num], speed_drive);
SoftPWMSetPercent(motorPins[mot_num + 4], 0);

}

void spin_reverse(byte speed_drive, byte mot_num){

SoftPWMSetPercent(motorPins[mot_num], 0);
SoftPWMSetPercent(motorPins[mot_num + 4], speed_drive);

}


void spin_all(byte speed_drive){

for(byte i = 0;i<4;i++){
SoftPWMSetPercent(motorPins[i], speed_drive);
SoftPWMSetPercent(motorPins[i+4], 0);
}

}

void spin_all_reverse(byte speed_drive){

for(byte i = 0;i<4;i++){
SoftPWMSetPercent(motorPins[i+4], speed_drive);
SoftPWMSetPercent(motorPins[i], 0);
}

}

void stop_all(){
  for(byte i = 0;i<8;i++){ SoftPWMSetPercent(motorPins[i], 0);}  
  }

void slide_right(byte speed_slide){

SoftPWMSetPercent(FLA, speed_slide);
SoftPWMSetPercent(FLB, 0);
SoftPWMSetPercent(RLA, 0);
SoftPWMSetPercent(RLB, speed_slide);

SoftPWMSetPercent(FRA, 0);
SoftPWMSetPercent(FRB, speed_slide);
SoftPWMSetPercent(RRA, speed_slide);
SoftPWMSetPercent(RRB, 0);
  
  }
  
void slide_left(byte speed_slide){

SoftPWMSetPercent(FLA, 0);
SoftPWMSetPercent(FLB, speed_slide);
SoftPWMSetPercent(RLA, speed_slide);
SoftPWMSetPercent(RLB, 0);

SoftPWMSetPercent(FRA, speed_slide);
SoftPWMSetPercent(FRB, 0);
SoftPWMSetPercent(RRA, 0);
SoftPWMSetPercent(RRB, speed_slide);
  
}

byte speed_spin = 12;

void setup() {
SoftPWMBegin();

  
for(byte i = 0;i<sizeof(motorPins);i++){pinMode(motorPins[i], OUTPUT);};

spin_all(speed_spin);
delay(1000);

stop_all();
delay(500);

spin_all_reverse(speed_spin);
delay(1000);
stop_all();
delay(500);

slide_left(15);
delay(1000);

stop_all();
delay(500);

slide_right(15);
delay(1000);
stop_all();

}

void loop() {


}
