

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 0);
  Serial.begin(9600);
}

void loop() {
 
  if (Serial.available() > 0) {
    byte led_status = Serial.read()-'0';
    Serial.println(led_status);
    digitalWrite(LED_BUILTIN, led_status);
  }
}
