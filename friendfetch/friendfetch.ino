/*
 * friendfetch - Arduino motor driver code for Rockie.
 * 
 * RPI Rock Raiders
 * 5/2/14
 *
 * Last Updated: Bryant Pong: 5/29/14 - 9:24 PM
 */
 
// AVR Libraries:
#include <avr/interrupt.h> // Support for interrupts
 
// ROS Libraries
#include <ros.h> // Core ROS Arduino Library
#include <std_msgs/String.h> // ROS std_msgs - String message (used for debugging messages)
#include <nav_msgs/Odometry.h> // ROS nav_msgs - Odometry message
#include <geometry_msgs/Twist.h> // ROS geometry_msgs - Twist message

// Arduino Libraries
#include <Servo.h>

/** CONSTANTS AND DEFINITIONS **/

/*
 * Pin Mappings:
 * Left Encoder: Digital Pin 20 (Interrupt 3)
 * Right Encoder: Digital Pin 21 (Interrupt 2)
 *
 * Rockie's Main Left Motor: Digital Pin 52
 * Rockie's Main Right Motor: Digital Pin 53
 *
 * Lifter Motor:
 * 
 */

const int LEFTENCODER = 20;
const int RIGHTENCODER = 21;

const int LEFTMOTOR = 52;
const int RIGHTMOTOR = 53;

const int POT = 50;
const int ENABLE = 45;
const int INPUT1 = 42;
const int INPUT2 = 22;

// Amount of time (in milliseconds) to raise/lower the scoop:
const int SCOOPTIME = 15000;

// Servo motor objects
Servo leftMotor, rightMotor;

// Volatile variables to hold the number of encoder ticks:
volatile int leftEncoderTicks = 0;
volatile int rightEncoderTicks = 0;

// Variables for the PI Velocity control loop:
const double kp = 0.1;
const double ki = 0.5;

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

/** END SECTION CONSTANTS AND DEFINITIONS **/

/** FUNCTION AND INTERRUPT PROTOTYPES **/

// Interrupt Prototypes:
void leftencoderinterrupt(); 
void rightencoderinterrupt(); 

// PI Motor Control Loop prototype: - IN PROGRESS
void piVelLoop(const double targetLinVel, const double targetAngVel);

// Functions for the Linear Actuator Scoop + Lifter:
void raiseScoop(void); // Implemented, not tested
void lowerScoop(void); // Implemented, not tested

void extendScoop(void); // Not implemented
void retractScoop(void); // Not implemented

// Callbacks:

// This callback is for receiving velocity commands from the SLAM navigation node:
void velCommandCallback(const geometry_msgs::Twist& nextVelocityCommand);

/** END SECTION FUNCTION AND INTERRUPT PROTOTYPES **/

/** ROS Objects **/
// ros::Subscriber<std_msgs::String> sub("arduino", &messageCb);

// The ROS Arduino Node object
ros::NodeHandle nh;

/*
 * ROS Publishers:
 *
 * debugChannel - This publisher publishes debug messages.
 * 
 */

// ROS Messages to Publish:
std_msgs::String debugMsg;
nav_msgs::Odometry odomData;

ros::Publisher debugChannel("debug_channel", &debugMsg);
ros::Publisher odometryData("odometry_data", &odomData);

/*
 * ROS Subscribers:
 *
 * velCommandSubscriber - This subscriber listens for geometry_msgs/Twist messages 
 *                        from the cmd_vel topic to drive Rockie.
 */
ros::Subscriber<geometry_msgs::Twist> velCommandSubscriber("cmd_vel", &velCommandCallback);
/** END SECTION ROS OBJECTS **/

/** INTERRUPTS **/

// Interrupts for encoders:
void leftencoderinterrupt(void) {
  leftEncoderTicks++;
} // End interrupt leftencoderinterrupt()

void rightencoderinterrupt(void) {
  rightEncoderTicks++;
} // End interrupt rightencoderinterrupt()

/** END SECTION INTERRUPTS **/

/** SCOOP AND LIFTER FUNCTIONS **/

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

/** END SECTION SCOOP AND LIFTER FUNCTIONS **/

// PI Loop function to control motor velocity:
void piVelLoop(double targetLinVel, double targetAngVel) {
  
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
 * This callback allows the SLAM Node to send Twist messages to Rockie.
 * When this callback is called, the PI Control loop will be called with 
 * the target linear and angular velocity.  
 *
 * Since this callback relies on a geometry_msgs::Twist message, the
 * linear velocity is in the Vector3 "linear" X datafield while the
 * angular velocity is in the Vector3 "angular" Z datafield.
 */
void velCommandCallback(const geometry_msgs::Twist& nextVelocityCommand) {
  
  // Let's get the designated linear and angular velocities:
  const double targetLinearVelocity = nextVelocityCommand.linear.x;
  const double targetAngularVelocity = nextVelocityCommand.angular.z;
  
  // Call the PI Velocity Control Loop with the target velocities:
  piVelLoop(targetLinearVelocity, targetAngularVelocity);
  
} // End callback velCommandCallback()

#ifdef DEBUG
    
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
#endif

/** SETUP Function **/

void setup() {
  
  
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
  //nh.initNode();
 
  // Advertise the debug and error channels:
  //nh.advertise(debugChannel);
  
  //nh.subscribe(velCommandSubscriber);
  
  // Initialize Rockie's main left and right motors:
  //leftMotor.attach(LEFTMOTOR);
  //rightMotor.attach(RIGHTMOTOR);
  
  // We want Rockie to brake when starting:
  //leftMotor.write(90);
  //rightMotor.write(90);
  
  //pinMode(POT, INPUT);
  //pinMode(ENABLE, OUTPUT);
  //pinMode(INPUT1, OUTPUT);
  //pinMode(INPUT2, OUTPUT);
  
  //digitalWrite(ENABLE, HIGH);
  
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
    
    Serial.begin(9600);
} 

/** END SETUP FUNCTION **/

/** LOOP Function **/
void loop() {
  
  Serial.println("Please enter the target linear velocity");
  while(!Serial.available());
  double targetLinearVelocity = Serial.parseFloat();
  Serial.println("The new target linear velocity is: ");
  Serial.println(targetLinearVelocity);
  Serial.println("Now enter the target angular velocity");
  while(!Serial.available());
  double targetAngularVelocity = Serial.parseFloat();
  Serial.println("The new target angular velocity is: ");
  Serial.println(targetAngularVelocity);
  
  Serial.println("Now beginning PI loop:");
  piVelLoop(targetLinearVelocity, targetAngularVelocity);
  
  // Have the ROS Nodes update themselves:
  //nh.spinOnce();
}

/** END LOOP FUNCTION **/
