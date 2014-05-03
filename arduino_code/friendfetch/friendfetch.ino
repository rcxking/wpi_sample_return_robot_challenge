/*
 * friendfetch - Arduino motor driver code for Rockie.
 * 
 * RPI Rock Raiders
 * 5/2/14
 *
 * Last Updated: Bryant Pong: 5/2/14 - 2:56 PM
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

// Volatile variables to hold the number of encoder ticks:
volatile int leftEncoderTicks = 0;
volatile int rightEncoderTicks = 0;

// Variables for the PD Velocity control loop:
double kp = 0.1;
double kd = 0.5;

// Variables to keep track of time elapsed since the PD Loop was last called:
unsigned long timeStart, timeStop;

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
void pdVelLoop(int motor, double targetLinVel, double targetAngVel) {
  
  // Let's get the number of left and right encoder ticks:
  double lEncTicks = leftEncoderTicks;
  double rEncTicks = rightEncoderTicks;

  // Reset the left and right encoder tick counts:
  leftEncoderTicks = rightEncoderTicks = 0;  
  
  // Calculate the linear and angular velocities of the encoders:
  double leftWheelAngularVel;
  double leftWheelLinearVel;
  
  
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
  } //else if( (strcmp("MOTOR"
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

Servo leftMotor, rightMotor;

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
  
  // Advertise the ROS Publishers for the Encoder Data:
  nh.advertise(leftEncoder);
  nh.advertise(rightEncoder);
  
  // Advertise the debug and error channels:
  nh.advertise(debugChannel);
  nh.advertise(errorChannel);
  
  nh.subscribe(sub);
   
  leftMotor.attach(LEFTMOTOR);
  rightMotor.attach(RIGHTMOTOR);
}

int pos = 0;

void loop() {
 
  // Have the ROS Nodes update themselves:
  nh.spinOnce();
 
  /* 
  Serial.println("leftEncoderTicks: ");
  Serial.println(leftEncoderTicks);
  
  Serial.println("rightEncoderTicks: ");
  Serial.println(rightEncoderTicks);
  */
  
  //delay(500);
  
  /*
  for(pos = 0; pos < 180; pos ++) {
   leftMotor.write(pos);
   delay(15);
  } 
  */
  
  /*
  char lev[10] = "";
  leftEncoderData.data = "";
  int leftEncoderValue = analogRead(LEFTENCODER);
  sprintf(leftEncoderData.data, "%d", leftEncoderValue);
  leftEncoder.publish(&leftEncoderData);
  
  delay(1);
  */
}
