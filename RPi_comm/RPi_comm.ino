
/*


void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 0);
  
}

void loop() {
 
  if (Serial.available() > 0) {
    byte led_status = Serial.read()-'0';
    
    switch(led_status){

      case 1:
        digitalWrite(LED_BUILTIN, 0);
        break;
      case 2:
        digitalWrite(LED_BUILTIN, 1);
        break;
      default:
      break;
      }
    
    
    
  }
}

*/
