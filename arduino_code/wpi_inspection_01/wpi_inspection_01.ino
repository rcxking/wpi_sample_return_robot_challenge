const int LEDPIN = 22;
const int PAUSEPINXBEE = 4;

unsigned long currentTime;
unsigned long oldTime = millis();
int ledOn = 0;

void setup() {
  pinMode(LEDPIN, OUTPUT);
  pinMode(PAUSEPINXBEE, INPUT);
  
  Serial.begin(9600);
}

void loop() {
 currentTime = millis();

 if(digitalRead(PAUSEPINXBEE) == LOW) {
   if( (currentTime - oldTime) >= 1000 ) {
    
     if(ledOn == 0) {
      ledOn = 1;
     } else {
      ledOn = 0;
     } 
   
     Serial.println("One second elapsed!");
     oldTime = currentTime;
    }
  
    if(ledOn) {
      digitalWrite(LEDPIN, HIGH);
    } else {
      digitalWrite(LEDPIN, LOW);
    } 
 } else {
   digitalWrite(LEDPIN, HIGH);
   ledOn = 1;
 }
}
