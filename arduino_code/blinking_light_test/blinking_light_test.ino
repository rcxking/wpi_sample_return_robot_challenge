const int LEDPIN = 22;

unsigned long currentTime;
unsigned long oldTime = millis();
int ledOn = 0;

void setup() {
 pinMode(LEDPIN, OUTPUT);
 
 Serial.begin(9600);
}

void loop() {
  // Get the currentTime: 
  currentTime = millis();
  
  Serial.println("currentTime is: ");
  Serial.println(currentTime);
  Serial.println("oldTime is: ");
  Serial.println(oldTime);
 
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
}
