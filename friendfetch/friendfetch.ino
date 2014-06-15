/*
 * friendfetch - Arduino code for Rockie.
 *
 * This code has 6 different responsibilities:
 * 1) PI Motor Controller for Motion
 * 2) Reading encoders via interrupts
 * 3) 
 * 
 * RPI Rock Raiders
 * 5/2/14
 *
 * Last Updated: Bryant Pong: 6/12/14 - 11:22 AM
 */
 
// Misc. Libraries:
#include <digitalWriteFast.h>
 
// ROS Libraries
#include <ros.h> // Core ROS Arduino Library
#include <std_msgs/String.h> // ROS std_msgs - String message (used for debugging messages)
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h> // ROS nav_msgs - Odometry message
#include <geometry_msgs/Twist.h> // ROS geometry_msgs - Twist message

// Arduino Libraries
#include <Servo.h>

/** CONSTANTS AND DEFINITIONS **/

/*
 * Pin Mappings:
 * Left Encoder Pin A: Digital Pin 18 (Interrupt 5)
 * Left Encoder Pin B: Digital Pin 19 (Interrupt 4)
 * Right Encoder Pin A: Digital Pin 20 (Interrupt 3)
 * Right Encoder Pin B: Digital Pin 21 (Interrupt 2)
 *
 * Rockie's Main Left Motor: Digital Pin 52
 * Rockie's Main Right Motor: Digital Pin 53
 *
 * Lifter Motor:
 */

const byte ENCODER_1_PIN_A = 18;
const byte ENCODER_1_PIN_B = 19;
const byte ENCODER_2_PIN_A = 20;
const byte ENCODER_2_PIN_B = 21;

const int LEFTMOTOR = 52;
const int RIGHTMOTOR = 53;

const int POT = 50;
const int ENABLE = 45;
const int INPUT1 = 42;
const int INPUT2 = 22;

// Amount of time (in milliseconds) to raise/lower the scoop:
const int SCOOPTIME = 15000;

// State of Pins A and B on the left and right encoders:
int ENCODER_1_A_SET = 0;
int ENCODER_1_B_SET = 0;
int ENCODER_2_A_SET = 0;
int ENCODER_2_B_SET = 0;

// Servo motor objects
Servo leftMotor, rightMotor;

// Variables to hold the number of encoder ticks:
long leftEncoderTicks = 0;
long rightEncoderTicks = 0;

// Variables for the PI Velocity control loop:
const double kpL = 0.1;
const double kpA = 0.1;
const double ki = 0.5;

const double motorCenter = 90;
const double motorRange = 60;

// Variables to keep track of time elapsed since the PD Loop was last called:
unsigned long timeStart = 0;
unsigned long timeStop = 0;

// Variables to keep track of our velocity error:
double linVelocityError = 0.0;
double angVelocityError = 0.0;

// Other variables for the PD loop:
double linearCommand = 0.0;
double angularCommand = 0.0;

double leftMotorCommand = 0;
double rightMotorCommand = 0;
double maxCommand = 0;

/** END SECTION CONSTANTS AND DEFINITIONS **/

/** FUNCTION AND INTERRUPT PROTOTYPES **/

// Interrupt Prototypes:
void encoder1PinChangeA(void); // Implemented, tested
void encoder1PinChangeB(void); // Implemented, tested
void encoder2PinChangeA(void); // Implemented, tested
void encoder2PinChangeB(void); // Implemented, tested

// PI Motor Control Loop prototype: - IN PROGRESS
void piVelLoop(const double targetLinVel, const double targetAngVel);

// Functions for the Linear Actuator Scoop + Lifter:
void raiseScoop(void); // Implemented, not tested
void lowerScoop(void); // Implemented, not tested

void extendScoop(void); // Not implemented
void retractScoop(void); // Not implemented

// Basic functions to move the motor:
void turnLeft(int duration);
void turnRight(int duration);
void forward(int duration);
void backward(int duration);
double sign(double num);

// Callbacks:

// This callback is for receiving velocity commands from the SLAM navigation node:
void velCommandCallback(const geometry_msgs::Twist& nextVelocityCommand);

/** END SECTION FUNCTION AND INTERRUPT PROTOTYPES **/

/** ROS Objects **/

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

// Left Encoder Interrupts:
void encoder1PinChangeA(void) {
  ENCODER_1_A_SET = digitalReadFast2(ENCODER_1_PIN_A) == HIGH;
  leftEncoderTicks += (ENCODER_1_A_SET != ENCODER_1_B_SET) ? -1 : +1;
} // End interrupt encoder1PinChangeA()

void encoder1PinChangeB(void) {
  ENCODER_1_B_SET = digitalReadFast2(ENCODER_1_PIN_B) == HIGH;
  leftEncoderTicks += (ENCODER_1_A_SET == ENCODER_1_B_SET) ? -1 : +1;
} // End interrupt encoder1PinChangeB()

// Right Encoder Interrupts:
void encoder2PinChangeA(void) {
  ENCODER_2_A_SET = digitalReadFast2(ENCODER_2_PIN_A) == HIGH;
  rightEncoderTicks += (ENCODER_2_A_SET != ENCODER_2_B_SET) ? +1 : -1;
} // End interrupt encoder2PinChangeA()

void encoder2PinChangeB(void) {
  ENCODER_2_B_SET = digitalReadFast2(ENCODER_2_PIN_B) == HIGH;
  rightEncoderTicks += (ENCODER_2_A_SET == ENCODER_2_B_SET) ? +1 : -1;
} // End interrupt encoder2PinChangeB()

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

/** MAIN MOTOR FUNCTIONS **/

void turnLeft(int duration) {
  
} // End function turnLeft()

void turnRight(int duration) {
  
} // End function turnRight()

void forward(int duration) {
  
} // End function foward()

void backward(int duration) {
  
} // End function backward()

/** END SECTION MAIN MOTOR FUNCTIONS **/

// PI Loop function to control motor velocity:
void piVelLoop(double targetLinVel, double targetAngVel) {
  
  // Let's get the number of left and right encoder ticks:
  long lEncTicks = leftEncoderTicks;
  long rEncTicks = rightEncoderTicks;
  
  // Reset the left and right encoder tick counts:
  leftEncoderTicks = rightEncoderTicks = 0;  
  
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

  // Clock when the encoder ticks were read:
  timeStop = micros();
  
  // How long ago was the last encoder reading?
  double timeDiff = (double(timeStop) - double(timeStart)) / 1000000;
  
  // How many encoder ticks per second?
  double leftEncoderTicksPerSecond = double(lEncTicks) / double(timeDiff);
  double rightEncoderTicksPerSecond = double(rEncTicks) / double(timeDiff);
  
  // Calculate the linear and angular velocities of the encoders:
  double leftWheelAngularVel = double(leftEncoderTicksPerSecond) / double(800) / 8.3 * 2 * PI;
  double rightWheelAngularVel = double(rightEncoderTicksPerSecond) / double(800) / 8.3 * 2 * PI;
  double leftWheelLinearVel = leftWheelAngularVel * 0.1525;
  double rightWheelLinearVel = rightWheelAngularVel * 0.1525;
  
  // Now we can calculate the current linear and angular velocity of Rockie: 
  double currentLinVel = (leftWheelLinearVel + rightWheelLinearVel) / 2.0;
  double currentAngVel = (rightWheelLinearVel - leftWheelLinearVel) / (0.71);
  
  Serial.println("currentLinVel: ");
  Serial.println(currentLinVel);
  Serial.println("currentAngVel: ");
  Serial.println(currentAngVel);

  // We can now calculate our error in velocities:
  linVelocityError = currentLinVel - targetLinVel;
  angVelocityError = currentAngVel - targetAngVel;
  
  // Controller input confined to the range +-1
  linearCommand = -kpL * linVelocityError;
  angularCommand = -kpA * angVelocityError;


  
  Serial.println("linearCommand: ");
  Serial.println(linearCommand);
  Serial.println("angularCommand: ");
  Serial.println(angularCommand);
  /*
  Serial.println("deltaAngular: ");
  Serial.println(deltaAngular);
  */
  maxCommand = abs(linearCommand) + abs(angularCommand);
  leftMotorCommand = linearCommand - angularCommand;
  rightMotorCommand = linearCommand + angularCommand;
  if(maxCommand > 1){
    leftMotorCommand/=maxCommand;
    rightMotorCommand/=maxCommand;
  } 
 
  
  leftMotor.write(motorCenter + motorRange * leftMotorCommand);
  rightMotor.write(motorCenter + motorRange * rightMotorCommand);

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

  /** ENCODERS INITIALIZATION **/
  
  pinMode(ENCODER_1_PIN_A, INPUT);
  digitalWrite(ENCODER_1_PIN_A, HIGH);
  pinMode(ENCODER_1_PIN_B, INPUT);
  digitalWrite(ENCODER_1_PIN_B, HIGH);
  
  ENCODER_1_A_SET = digitalRead(ENCODER_1_PIN_A);
  ENCODER_1_B_SET = digitalRead(ENCODER_1_PIN_B);
  
  pinMode(ENCODER_2_PIN_A, INPUT);
  digitalWrite(ENCODER_2_PIN_A, HIGH);
  pinMode(ENCODER_2_PIN_B, INPUT);
  digitalWrite(ENCODER_2_PIN_B, HIGH);
  
  ENCODER_2_A_SET = digitalRead(ENCODER_2_PIN_A);
  ENCODER_2_B_SET = digitalRead(ENCODER_2_PIN_B);
  
  timeStart = micros();
  attachInterrupt(3, encoder2PinChangeA, CHANGE);
  attachInterrupt(2, encoder2PinChangeB, CHANGE);
  attachInterrupt(5, encoder1PinChangeA, CHANGE); 
  attachInterrupt(4, encoder1PinChangeB, CHANGE);
  
  Serial.begin(9600);
  
  /** END ENCODERS INITIALIZATION **/

  // Rockie should not move in the beginning:
  leftMotor.attach(LEFTMOTOR);
  rightMotor.attach(RIGHTMOTOR);

  leftMotor.write(90);
  rightMotor.write(90);
    
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
} 

/** END SETUP FUNCTION **/

/** LOOP Function **/
void loop() {

  leftMotor.write(120);
  rightMotor.write(120);
  //piVelLoop(0, 0);
  //delay(1000);
  // Have the ROS Nodes update themselves:
  //nh.spinOnce();
}

double sign(double num)
{
  return (num > 0) ? 1 : -1;
}

/** END LOOP FUNCTION **/
