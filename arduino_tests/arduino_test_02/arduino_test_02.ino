/*
 * arduino_test_02 - Tests for the scoop.
 *
 */
 
// Pin definitions for the lifter motor:
const int POT = 50;
const int ENABLE = 45;
const int INPUT1 = 42;
const int INPUT2 = 22;

// Pin definitions for the scoop motor:
const int SCOOPPOT = 51;
const int SCOOPENABLE = 46;
const int SCOOPINPUT1 = 43;
const int SCOOPINPUT2 = 23;

// Amount of time (in milliseconds) to raise/lower the scoop:
const int SCOOPTIME = 15000;

void raiseScoop(void) {
  digitalWrite(INPUT1, HIGH);
  digitalWrite(INPUT2, LOW);
  delay(SCOOPTIME);
  
  // Stop the Scoop:
  digitalWrite(INPUT1, LOW);
  digitalWrite(INPUT2, LOW);
} // End function raiseScoop();

void lowerScoop(void) {
  digitalWrite(INPUT1, LOW);
  digitalWrite(INPUT2, HIGH);
  delay(SCOOPTIME);
  
  // Stop the Scoop:
  digitalWrite(INPUT1, LOW);
  digitalWrite(INPUT2, LOW);
  
} // End function lowerScoop();

void extendScoop(void) {
  digitalWrite(SCOOPINPUT1, HIGH);
  digitalWrite(SCOOPINPUT2, LOW);
  delay(SCOOPTIME);
  
  // Stop the Scoop:
  digitalWrite(SCOOPINPUT1, LOW);
  digitalWrite(SCOOPINPUT2, LOW);
}

void retractScoop(void) {
  digitalWrite(SCOOPINPUT1, LOW);
  digitalWrite(SCOOPINPUT2, HIGH);
  delay(SCOOPTIME);
  
  // Stop the Scoop:
  digitalWrite(SCOOPINPUT1, LOW);
  digitalWrite(SCOOPINPUT2, LOW);
}

void setup() {
  pinMode(POT, INPUT);
  pinMode(ENABLE, OUTPUT);
  pinMode(INPUT1, OUTPUT);
  pinMode(INPUT2, OUTPUT);
  digitalWrite(ENABLE, HIGH);
  
  pinMode(SCOOPPOT, INPUT);
  pinMode(SCOOPENABLE, OUTPUT);
  pinMode(SCOOPINPUT1, OUTPUT);
  pinMode(SCOOPINPUT2, OUTPUT);
  
  digitalWrite(SCOOPENABLE, HIGH);
}

void loop() {
 lowerScoop();
 extendScoop(); 
 delay(2000);
 retractScoop();
 raiseScoop();
 delay(2000);
}
