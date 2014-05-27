/*
 * friendfetch - Arduino motor driver code for Rockie.
 * 
 * RPI Rock Raiders
 * 5/2/14
 *
 * Last Updated: Bryant Pong: 5/14/14 - 3:24 PM
 */
 
// AVR Libraries:
#include <avr/interrupt.h>
 
// ROS Libraries
#include <ros.h>
#include <std_msgs/String.h>

// Arduino Libraries
#include <Servo.h>

/*
 * Pin Mappings:
 * Left Encoder: Digital Pin 20 (Interrupt 3)
 * Right Encoder: Digital Pin 21 (Interrupt 2)
 */

const int LEFTENCODER = 20;
const int RIGHTENCODER = 21;

const int LEFTMOTOR = 52;
const int RIGHTMOTOR = 53;

const int POT = 50;
const int ENABLE = 45;
const int INPUT1 = 42;
const int INPUT2 = 22;

Servo leftMotor, rightMotor;

// Volatile variables to hold the number of encoder ticks:
volatile int leftEncoderTicks = 0;
volatile int rightEncoderTicks = 0;

// Variables for the PD Velocity control loop:
double kp = 0.1;
double kd = 0.5;

// Variables to keep track of time elapsed since the PD Loop was last called:
unsigned long timeStart = 0;
unsigned long timeStop = 0;

// Variables to keep track of our velocity error:
double linVelocityError = 0.0;
double angVelocityError = 0.0;

// Other variables for the PD loop:
double deltaLinear = 0.0;
double deltaAngular = 0.0;

double currentLeftVelocity = 91.0;
double currentRightVelocity = 91.0;

// The ROS NodeHandler for the Arduino.
ros::NodeHandle nh;

// ROS Publisher for debug and error messages:
std_msgs::String debugMsg;
ros::Publisher debugChannel("debug_channel", &debugMsg);

/*** INTERRUPTS ***/

// Interrupts for encoders:
void leftencoderinterrupt() {
  leftEncoderTicks++;
} // End interrupt leftencoderinterrupt()

void rightencoderinterrupt() {
  rightEncoderTicks++;
} // End interrupt rightencoderinterrupt()

/*** END SECTION INTERRUPTS **/

// PD Loop function to control motor velocity:
void pdVelLoop(double targetLinVel, double targetAngVel, int power) {
  
  // Let's get the number of left and right encoder ticks:
  double lEncTicks = leftEncoderTicks;
  double rEncTicks = rightEncoderTicks;
  
  // Flag to determine if we're going forward or backward:
  boolean forward = true;
  if(targetLinVel < 0) {
    forward = false;
  } // End if
  
  Serial.println("forward is: ");
  Serial.println(forward);
  
  Serial.println("lEncTicks");
  Serial.println(lEncTicks);
  Serial.println("rEncTicks");
  Serial.println(rEncTicks);

  // Reset the left and right encoder tick counts:
  leftEncoderTicks = rightEncoderTicks = 0;  
  
  // Clock when the encoder ticks were read:
  timeStop = millis();
  
  // How long ago was the last encoder reading?
  double timeDiff = (timeStop - timeStart) / 1000;
  
  // How many encoder ticks per second?
  double leftEncoderTicksPerSecond = double(lEncTicks) / double(timeDiff);
  double rightEncoderTicksPerSecond = double(rEncTicks) / double(timeDiff);
  
  // Calculate the linear and angular velocities of the encoders:
  double leftWheelAngularVel = double(leftEncoderTicksPerSecond) / double(400) / 8.3 * 2 * PI;
  double rightWheelAngularVel = double(rightEncoderTicksPerSecond) / double(400) / 8.3 * 2 * PI;
  double leftWheelLinearVel = leftWheelAngularVel * 0.1525;
  double rightWheelLinearVel = rightWheelAngularVel * 0.1525;
  
  // Now we can calculate the current linear and angular velocity of Rockie:
  double currentLinVel;
  if(forward) {
    currentLinVel = (leftWheelLinearVel + rightWheelLinearVel) / 2.0;
  } else {
    currentLinVel = -1 * ((leftWheelLinearVel + rightWheelLinearVel) / 2.0);
  } // End else
  double currentAngVel = (rightWheelAngularVel - leftWheelAngularVel) / (0.5 * 0.71);
  
  Serial.println("currentLinVel: ");
  Serial.println(currentLinVel);
  
  Serial.println("currentAngVel: ");
  Serial.println(currentAngVel);
  
  // We can now calculate our error in velocities:
  linVelocityError = currentLinVel - targetLinVel;
  angVelocityError = currentAngVel - targetAngVel;
  
  // Next we have to calculate the change in linear and angular velocity:
  deltaLinear = kp * linVelocityError;
  deltaAngular = kp * angVelocityError;
  
  Serial.println("deltaLinear: ");
  Serial.println(deltaLinear);
  /*
  Serial.println("deltaAngular: ");
  Serial.println(deltaAngular);
  */
  
  currentLeftVelocity = currentLeftVelocity + double(deltaLinear);
  currentRightVelocity = currentLeftVelocity + double(deltaLinear);
    
  if(currentLeftVelocity > 120) {
    currentLeftVelocity = 120.0;
  }
  if(currentLeftVelocity < 60) {
    currentLeftVelocity = 60;
  }
  if(currentRightVelocity > 120) {
    currentRightVelocity = 120.0;
  }
  if(currentRightVelocity < 60) {
    currentRightVelocity = 60;
  }
  
  Serial.println("currentLeftVelocity: ");
  Serial.println(currentLeftVelocity);
  Serial.println("currentRightVelocity: ");
  Serial.println(currentRightVelocity);
  
  
  leftMotor.write(currentLeftVelocity);
  rightMotor.write(currentRightVelocity);
  
#ifdef DEBUG
  currentLeftVelocity + 90 * deltaAngular;
  currentRightVelocity + 90 * deltaAngular;
  
  leftMotor.write(currentLeftVelocity);
  rightMotor.write(currentRightVelocity);
#endif
  
  // Now set our start time to be our end time:
  timeStart = timeStop;
} // End PD Loop

/*
 *  Callback for the ROS Publisher node.
 */
void messageCb(const std_msgs::String& nextCommand) {
  
  
  // Store the actual command from the nextCommand message:
  char nextCommandMsg[1024];
  strcpy(nextCommandMsg, nextCommand.data);
  
  // Parse the nextCommandMsg using string tokenization:
  char *nextWord;
  
  // String array to hold each word of the next command:
  char **commandWords = (char**) malloc(sizeof(char *) * 10);
  int commandWordsSize = 10;
  int nextWordPos = 0;
  for(int i = 0; i < commandWordsSize; i++) {
    char *temp = (char *) malloc(sizeof(char) * 100);
    commandWords[i] = temp;
  } // End for
  
  nextWord = strtok(nextCommandMsg, " ");
  while(nextWord != NULL) { 
    // Add the next command to the commandWords array:
    strcpy(commandWords[nextWordPos], nextWord);
    nextWordPos++;
    nextWord = strtok(NULL, " ");
  } // End while
  
  // Now that we got the commands, let's go and parse them!
  debugMsg.data = "Now parsing commands";
  debugChannel.publish(&debugMsg);
  if( (strcmp("SET", commandWords[0]) == 0)) {
    
    /*
     * We're expecting the SET command to have the following syntax:
     *
     * SET <new left motor velocity> <new right motor velocity>
     */
     
    currentLeftVelocity = atof(commandWords[1]);
    currentRightVelocity = atof(commandWords[2]);
    
    debugMsg.data = "SET ";
    strcat(debugMsg.data, commandWords[1]);
    strcat(debugMsg.data, " ");
    strcat(debugMsg.data, commandWords[2]);
    debugChannel.publish(&debugMsg);
    
  } else if(strcmp("DRIVE", commandWords[0]) == 0) {
    
    /*
     * This is a MOTOR message asking for a change in velocity to the motors:
     *
     * A motor message has the following form:
     * MOTOR <New Linear Velocity> <New Angular Velocity>
     */
    
    char linVel[10];
    char angVel[10];
    char duration[10];
    
    strcpy(linVel, commandWords[1]);
    strcpy(angVel, commandWords[2]);
    strcpy(duration, commandWords[3]);
    
    double linVelo = atof(linVel);
    double angVelo = atof(angVel);
    double dur = atof(duration);
    
    leftMotor.write(currentLeftVelocity);
    rightMotor.write(currentRightVelocity);
    delay(dur);
    
    debugMsg.data = "linVel: ";
    debugChannel.publish(&debugMsg);
    strcpy(debugMsg.data, linVel);
    debugChannel.publish(&debugMsg);
    debugMsg.data = "angVel: ";
    debugChannel.publish(&debugMsg);
    strcpy(debugMsg.data, angVel);
    debugChannel.publish(&debugMsg);
    
  } else if(strcmp("GO", commandWords[0]) == 0) {
    debugMsg.data = "This is a GO Command";
    debugChannel.publish(&debugMsg);  
    
    // Go forward 5 meters at 0.3333 m/sec:
    leftMotor.write(50);
    rightMotor.write(60);
    delay(5000);
    
    leftMotor.write(90);
    rightMotor.write(90);
    
    /*
    digitalWrite(INPUT1, LOW);
    digitalWrite(INPUT2, HIGH);
    delay(10);
    */
    
    leftMotor.write(40);
    rightMotor.write(30);
    delay(15000);
    
    // Stop the motors after we're done:
    leftMotor.write(90);
    rightMotor.write(90);    
  } else if(strcmp("UP", commandWords[0]) == 0) {
    digitalWrite(INPUT1, HIGH);
    digitalWrite(INPUT2, LOW);
    delay(2000);
  } else if(strcmp("DOWN", commandWords[0]) == 0) {
    digitalWrite(INPUT1, LOW);
    digitalWrite(INPUT2, HIGH);
    delay(2000);
  }
  debugMsg.data = "Done parsing command";
  debugChannel.publish(&debugMsg);
  
  // Good policy to deallocate dynmically allocated memory:
  free(nextWord);
  
  // Free up the array of the next command words:
  for(int i = 0; i < commandWordsSize; i++) {
    free(commandWords[i]);
  } // End for
  free(commandWords);
} // End callback messageCb()

ros::Subscriber<std_msgs::String> sub("arduino", &messageCb);

void setup() {
  //Serial.begin(9600);
  
  // The Encoders are set as inputs:
  pinMode(LEFTENCODER, INPUT);
  pinMode(RIGHTENCODER, INPUT);
  
  /*
   * Attach an interrupt for the left encoder to count how many ticks
   * the left and right encoders counted:
   */
  attachInterrupt(3, leftencoderinterrupt, CHANGE);
  attachInterrupt(2, rightencoderinterrupt, CHANGE);
    
  // Start the ROS Node:
  nh.initNode();
 
  // Advertise the debug and error channels:
  nh.advertise(debugChannel);
  
  nh.subscribe(sub);
  
  leftMotor.attach(LEFTMOTOR);
  rightMotor.attach(RIGHTMOTOR);
  
  leftMotor.write(90);
  rightMotor.write(90);
  
  //Serial.begin(9600);
  
  pinMode(POT, INPUT);
  pinMode(ENABLE, OUTPUT);
  pinMode(INPUT1, OUTPUT);
  pinMode(INPUT2, OUTPUT);
  
  digitalWrite(ENABLE, HIGH);
  
  /*
  // Go forward 5 meters at 0.3333 m/sec:
    leftMotor.write(130);
    rightMotor.write(120);
    delay(15000);
    
    for(int i = 130; i > 90; i -= 10) {
      leftMotor.write(i);
      rightMotor.write(i);
      delay(250);
    }
    
    leftMotor.write(90);
    rightMotor.write(90);
    delay(1000);
    
    
    digitalWrite(INPUT1, LOW);
    digitalWrite(INPUT2, HIGH);
    delay(16000);
    
    digitalWrite(INPUT1, HIGH);
    digitalWrite(INPUT2, LOW);
    delay(16000);
    
    leftMotor.write(60);
    rightMotor.write(50);
    delay(15000);
    
    for(int i = 50; i < 90; i+=10) {
      leftMotor.write(i);
      rightMotor.write(i);
      delay(250);
    }
    
    // Stop the motors after we're done:
    leftMotor.write(90);
    rightMotor.write(90);   
    delay(1000);*/
    
    
}

void loop() {
  
  // Have the ROS Nodes update themselves:
  nh.spinOnce();
}
