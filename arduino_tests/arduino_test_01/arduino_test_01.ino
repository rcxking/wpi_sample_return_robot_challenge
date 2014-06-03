/*
 * arduino_test_01 - Test for the new Arduino's encoders.
 *
 * RPI Rock Raiders
 * 6/2/14
 *
 * Last Updated: Bryant Pong 6/2/14 - 11:04 PM
 */

 
/** Constants and Definitions **/
/*
 * Pin Mappings:
 * Left Encoder Pin A: Digital Pin 20 (Interrupt 3)
 * Left Encoder Pin B: Digital Pin 24
 * Right Encoder Pin A: Digital Pin 21 (Interrupt 2)
 * Right Encoder Pin B: Digital Pin 25
 * 
 * Refer to:
 * http://www.hessmer.org/blog/2011/01/30/quadrature-encoder-too-fast-for-arduino/
 * for more code on the interrupts.
 */
const int LEFTENCODERA = 20;
const int LEFTENCODERB = 24;
const int RIGHTENCODERA = 21;
const int RIGHTENCODERB = 25;

// Variables to keep track of encoder ticks:
volatile int leftEncoderTicks = 0;
volatile int rightEncoderTicks = 0;
 
/** END SECTION CONSTANTS AND DEFINITIONS **/
 
/** Interrupts: **/
void leftEncoderInterrupt(void) {
  if(digitalRead(LEFTENCODER
} // End interrupts leftEncoderInterrupt()
 
/** END SECTION INTERRUPTS **/
 
// Setup function:
void setup() {
   // Start the Serial Console at 9600 Baud:
   Serial.begin(9600);
   
   // Enable pull-up resistors for the encoders:
   pinMode(LEFTENCODERA, INPUT);
   digitalWrite(LEFTENCODERA, LOW);
   pinMode(LEFTENCODERB, INPUT);
   digitalWrite(LEFTENCODERB, LOW);
   
} // End setup()
 
// Loop function:
void loop() {
   
} // End loop()
 
