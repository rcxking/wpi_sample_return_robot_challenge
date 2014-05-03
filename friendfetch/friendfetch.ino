/*
 * friendfetch - Arduino motor driver code for Rockie.
 * 
 * RPI Rock Raiders
 * 5/2/14
 *
 * Last Updated: Bryant Pong: 5/3/14 - 12:28 PM
 */
 
// AVR Libraries:
#include <avr/interrupt.h>
 
// ROS Libraries
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

// Arduino Libraries
#include <Servo.h>

/*
 * Pin Mappings:
 * Left Encoder: Digital Pin 20 (Interrupt 3)
 * Right Encoder: Digital Pin 21 (Interrupt 2)
 */

#define LEFTENCODER 20
#define RIGHTENCODER 21

#define LEFTMOTOR 52
#define RIGHTMOTOR 53

Servo leftMotor, rightMotor;

// Volatile variables to hold the number of encoder ticks:
volatile int leftEncoderTicks = 0;
volatile int rightEncoderTicks = 0;

// Variables for the PD Velocity control loop:
double kp = 10.0;
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

double currentLeftVelocity = 90.0;
double currentRightVelocity = 90.0;

// The ROS NodeHandler for the Arduino.
ros::NodeHandle nh;

// These are the ROS Publishers for the encoder data.  One channel per encoder.
std_msgs::String leftEncoderData, rightEncoderData;
ros::Publisher leftEncoder("left_encoder_data", &leftEncoderData);
ros::Publisher rightEncoder("right_encoder_data", &rightEncoderData);

// ROS Publisher for debug and error messages:
std_msgs::String debugMsg, errorMsg;
ros::Publisher debugChannel("debug_channel", &debugMsg);
ros::Publisher errorChannel("error_channel", &errorMsg);

// Interrupts for encoders:
void leftencoderinterrupt() {
  leftEncoderTicks++;
} // End interrupt leftencoderinterrupt()

void rightencoderinterrupt() {
  rightEncoderTicks++;
} // End interrupt rightencoderinterrupt()

// PD Loop function to control motor velocity:
void pdVelLoop(double targetLinVel, double targetAngVel) {
  
  // Let's get the number of left and right encoder ticks:
  double lEncTicks = leftEncoderTicks;
  double rEncTicks = rightEncoderTicks;
  
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
  double currentLinVel = (leftWheelLinearVel + rightWheelLinearVel) / 2.0;
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
  
  Serial.println("deltaAngular: ");
  Serial.println(deltaAngular);
  
  currentLeftVelocity = 90 + 90 * deltaLinear;
  currentRightVelocity = 90 + 90 * deltaLinear;
    
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
  rightMotor.write(currentLeftVelocity);
  
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
 
  // Log the next message received in the DEBUG channel:
  char buffer[1024] = "[DEBUG CHANNEL] Received msg: ";
  strcat(buffer, nextCommand.data);
  debugMsg.data = buffer;
  debugChannel.publish(&debugMsg);
  
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
  
  debugMsg.data = "Now tokenizing command: ";
  debugChannel.publish(&debugMsg);
  
  nextWord = strtok(nextCommandMsg, " ");
  while(nextWord != NULL) { 
    // Add the next command to the commandWords array:
    //strcpy(temp, nextWord);
    strcpy(commandWords[nextWordPos], nextWord);
    nextWordPos++;
    //free(temp);
    
    sprintf(debugMsg.data, "%d", nextWordPos);
    debugChannel.publish(&debugMsg);
    
    //debugMsg.data = "Now getting next word";
    //debugChannel.publish(&debugMsg);
    nextWord = strtok(NULL, " ");
  } // End while
  
  // Now that we got the commands, let's go and parse them!
  debugMsg.data = "Now parsing commands";
  debugChannel.publish(&debugMsg);
  if( (strcmp("GET", commandWords[0]) == 0)) {
    debugMsg.data = "This is a GET message!";
    debugChannel.publish(&debugMsg);
      
    // We're expecting that the command word after the "GET" is "leftEncoderData" or "rightEncoderData":
    if(strcmp("leftEncoderData", commandWords[1]) == 0) {
      char lev[10] = "";
      leftEncoderData.data = "";
      sprintf(leftEncoderData.data, "%d", leftEncoderTicks);
      leftEncoder.publish(&leftEncoderData);
    } else {
      char lev[10] = "";
      rightEncoderData.data = "";
      sprintf(rightEncoderData.data, "%d", rightEncoderTicks);
      rightEncoder.publish(&rightEncoderData);
    } // End else
  } else if(strcmp("MOTOR", commandWords[0]) == 0) {
    
    /*
     * This is a MOTOR message asking for a change in velocity to the motors:
     *
     * A motor message has the following form:
     * MOTOR <New Linear Velocity> <New Angular Velocity>
     */
    
    char linVel[10];
    char angVel[10];
    
    strcpy(linVel, commandWords[1]);
    strcpy(angVel, commandWords[2]);
    
    double linVelo = atof(linVel);
    double angVelo = atof(angVel);
    
    debugMsg.data = "linVel: ";
    debugChannel.publish(&debugMsg);
    strcpy(debugMsg.data, linVel);
    debugChannel.publish(&debugMsg);
    debugMsg.data = "angVel: ";
    debugChannel.publish(&debugMsg);
    strcpy(debugMsg.data, angVel);
    debugChannel.publish(&debugMsg);
    
    pdVelLoop(linVelo, angVelo);
    
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
    
   /*
  // Start the ROS Node:
  nh.initNode();
  
  // Advertise the ROS Publishers for the Encoder Data:
  nh.advertise(leftEncoder);
  nh.advertise(rightEncoder);
  
  // Advertise the debug and error channels:
  nh.advertise(debugChannel);
  nh.advertise(errorChannel);
  
  nh.subscribe(sub);
  */
   
  leftMotor.attach(LEFTMOTOR);
  rightMotor.attach(RIGHTMOTOR);
  
  leftMotor.write(90);
  rightMotor.write(90);
  
  Serial.begin(9600);
}

void loop() {
 
  //leftMotor.write(120);
  //rightMotor.write(120);
  
  // Have the ROS Nodes update themselves:
  nh.spinOnce();
  
  pdVelLoop(0.5, 0);
  delay(1000);
}
