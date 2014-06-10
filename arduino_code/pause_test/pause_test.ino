const int PAUSEPINXBEE = 4;

void setup() {
  Serial.begin(9600);  
  pinMode(PAUSEPINXBEE, INPUT);
}

void loop() {
  if(digitalRead(PAUSEPINXBEE) == HIGH) {
    Serial.println("Arduino paused");
  } else {
    Serial.println("ARDUINO NOT PAUSED");
  }
}


