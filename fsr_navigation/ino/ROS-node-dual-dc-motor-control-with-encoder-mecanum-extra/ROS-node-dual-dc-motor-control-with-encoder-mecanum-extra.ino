//Title: DC Motor Control with Encoder (Single)
//Author: Khairul Izwan 10-03-2020
//Description: Controlling DC Motor with Encoder using

//Parts
//1. DC Motor
//:: 12V 38RPM 5kgfcm Brushed DC Geared Motor with Encoder 
//:: https://my.cytron.io/p-12v-38rpm-5kgfcm-brushed-dc-geared-motor-with-encoder?search=12v%2038RPM&description=1&src=search.list
//2. Driver
//:: Shield L298P Motor Driver with GPIO 
//:: https://my.cytron.io/p-shield-l298p-motor-driver-with-gpio?src=search.list

//include necessary library
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

//Motor Pin Assignment
int RIGHT_DIR1 = 2;
int RIGHT_PWM1 = 3;
int RIGHT_DIR2 = 4;
int RIGHT_PWM2 = 5;
int LEFT_DIR1 = 9;
int LEFT_PWM1 = 8;
int LEFT_DIR2 = 7;
int LEFT_PWM2 = 6;

//Speed
int SPD1 = 0;

//Encoder Pins Definition
//Using an interrupt pins 
//:: 2, 3, 18, 19, 20, 21 
//:: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/?setlang=it
#define ENCODER_COUNT_UP_RIGHT 19
#define ENCODER_COUNT_DOWN_RIGHT 18
#define ENCODER_COUNT_UP_LEFT 21
#define ENCODER_COUNT_DOWN_LEFT 20

//Encoder Variables
volatile signed int TEMP_A, COUNTER_A = 0;
volatile signed int TEMP_B, COUNTER_B = 0; 

//Change according to the robot wheel dimension
#define wheelSep 0.38 // mm
#define wheelRadius 0.152; // mm

//Variables declaration
float transVelocity;
float rotVelocity;

float leftVelocity;
float rightVelocity;

float leftDutyCycle;
float rightDutyCycle;

float leftPWM=20;
float rightPWM=20;

ros::NodeHandle nh;

//Counter Helper Function*/
//Count Up A
void COUNT_INTERRUPT_CW_A() 
{
  if(digitalRead(ENCODER_COUNT_DOWN_RIGHT)==LOW) 
  {
    COUNTER_A++;
  }
  else
  {
    COUNTER_A--;
  }
}

//Count Down A
void COUNT_INTERRUPT_CCW_A()
{
  if(digitalRead(ENCODER_COUNT_UP_RIGHT)==LOW) 
  {
    COUNTER_A--;
  }
  else
  {
    COUNTER_A++;
  }
}

//Count Up B
void COUNT_INTERRUPT_CW_B() 
{
  if(digitalRead(ENCODER_COUNT_DOWN_LEFT)==LOW) 
  {
    COUNTER_B++;
  }
  else
  {
    COUNTER_B--;
  }
}

//Count Down A
void COUNT_INTERRUPT_CCW_B()
{
  if(digitalRead(ENCODER_COUNT_UP_LEFT)==LOW) 
  {
    COUNTER_B--;
  }
  else
  {
    COUNTER_B++;
  }
}

//Callback function for geometry_msgs::Twist
void messageCb_cmd_vel(const geometry_msgs::Twist &msg)
{
//  Get the ros topic value
  transVelocity = msg.linear.x;
  rotVelocity = msg.angular.z;
  
//  Differential Drive Kinematics
//::http://www.cs.columbia.edu/~allen/F15/NOTES/icckinematics.pdf
//  Differential Drive Kinematics
//::https://snapcraft.io/blog/your-first-robot-the-driver-4-5

//  Step 1: Calculate wheel speeds from Twist
  leftVelocity = transVelocity - ((rotVelocity * wheelSep) / 2);
  rightVelocity = transVelocity + ((rotVelocity * wheelSep) / 2);
  
//  Step 2: Convert wheel speeds into duty cycles
  leftDutyCycle = (255 * leftVelocity) / 0.22;
  rightDutyCycle = (255 * rightVelocity) / 0.22;
  
//  Step3: Ensure DutyCycle is between minimum and maximum
  leftPWM = clipPWM(abs(leftDutyCycle), 20, 200);
  rightPWM = clipPWM(abs(rightDutyCycle), 20, 200);

//  motor directection helper function
//  motorDirection();
}

//Helper function to ensure DutyCycle is between minimum
//and maximum
float clipPWM(float PWM, float minPWM, float maxPWM)
{
  if (PWM < minPWM)
  {
    return minPWM;
  }
  else if (PWM > maxPWM)
  {
    return maxPWM;
  }
  return PWM;
}

//Motor Direction helper function
void motorDirection()
{
  if (leftDutyCycle > 0 and rightDutyCycle > 0)
  {
  //Forward
    digitalWrite(RIGHT_DIR1, HIGH);
    digitalWrite(RIGHT_DIR2, LOW);
    digitalWrite(LEFT_DIR1, LOW);
    digitalWrite(LEFT_DIR2, HIGH);
  //PWM Speed Control
    analogWrite(RIGHT_PWM1, rightPWM);
    analogWrite(RIGHT_PWM2, rightPWM);
    analogWrite(LEFT_PWM1, leftPWM);
    analogWrite(LEFT_PWM2, leftPWM);
  }
  else if (leftDutyCycle < 0 and rightDutyCycle < 0)
  {
  //Backward
    digitalWrite(RIGHT_DIR1, LOW);
    digitalWrite(RIGHT_DIR2, HIGH);
    digitalWrite(LEFT_DIR1, HIGH);
    digitalWrite(LEFT_DIR2, LOW);
  //PWM Speed Control
    analogWrite(RIGHT_PWM1, rightPWM);
    analogWrite(RIGHT_PWM2, rightPWM);
    analogWrite(LEFT_PWM1, leftPWM);
    analogWrite(LEFT_PWM2, leftPWM);
  }
  else if (leftDutyCycle < 0 and rightDutyCycle > 0)
  {
  //Rotate Left
    digitalWrite(RIGHT_DIR1, HIGH);
    digitalWrite(RIGHT_DIR2, LOW);
    digitalWrite(LEFT_DIR1, HIGH);
    digitalWrite(LEFT_DIR2, LOW);
  //PWM Speed Control
    analogWrite(RIGHT_PWM1, rightPWM);
    analogWrite(RIGHT_PWM2, rightPWM);
    analogWrite(LEFT_PWM1, leftPWM);
    analogWrite(LEFT_PWM2, leftPWM);
  }
  else if (leftDutyCycle > 0 and rightDutyCycle < 0)
  {
  //Rotate Right
    digitalWrite(RIGHT_DIR1, LOW);
    digitalWrite(RIGHT_DIR2, HIGH);
    digitalWrite(LEFT_DIR1, LOW);
    digitalWrite(LEFT_DIR2, HIGH);
  //PWM Speed Control
    analogWrite(RIGHT_PWM1, rightPWM);
    analogWrite(RIGHT_PWM2, rightPWM);
    analogWrite(LEFT_PWM1, leftPWM);
    analogWrite(LEFT_PWM2, leftPWM);
  }
  else if (leftDutyCycle == 0 and rightDutyCycle == 0)
  {
  //PWM Speed Control
    analogWrite(RIGHT_PWM1, 0);
    analogWrite(RIGHT_PWM2, 0);
    analogWrite(LEFT_PWM1, 0);
    analogWrite(LEFT_PWM2, 0);
  }
}

//Encoder reset helper function
void messageCb_reset_encLeft(const std_msgs::Bool &msg)
{
  if (msg.data == true)
  {
    COUNTER_A = 0;
  }
}

//Encoder reset helper function
void messageCb_reset_encRight(const std_msgs::Bool &msg)
{
  if (msg.data == true)
  {
    COUNTER_B = 0;
  }
}

//AX-12 motorDirection helper function
void messageCb_ax12_control(const std_msgs::Int32 &msg)
{
  if (msg.data == 1)
  {
  //Forward
    digitalWrite(RIGHT_DIR1, HIGH);
    digitalWrite(RIGHT_DIR2, LOW);
    digitalWrite(LEFT_DIR1, LOW);
    digitalWrite(LEFT_DIR2, HIGH);
  //PWM Speed Control
    analogWrite(RIGHT_PWM1, rightPWM);
    analogWrite(RIGHT_PWM2, rightPWM);
    analogWrite(LEFT_PWM1, leftPWM);
    analogWrite(LEFT_PWM2, leftPWM);
  }
  else if (msg.data == 2)
  {
  //Backward
    digitalWrite(RIGHT_DIR1, LOW);
    digitalWrite(RIGHT_DIR2, HIGH);
    digitalWrite(LEFT_DIR1, HIGH);
    digitalWrite(LEFT_DIR2, LOW);
  //PWM Speed Control
    analogWrite(RIGHT_PWM1, rightPWM);
    analogWrite(RIGHT_PWM2, rightPWM);
    analogWrite(LEFT_PWM1, leftPWM);
    analogWrite(LEFT_PWM2, leftPWM);
  }
  else if (msg.data == 3)
  {
  //Left
    digitalWrite(RIGHT_DIR1, HIGH);
    digitalWrite(RIGHT_DIR2, HIGH);
    digitalWrite(LEFT_DIR1, HIGH);
    digitalWrite(LEFT_DIR2, HIGH);
  //PWM Speed Control
    analogWrite(RIGHT_PWM1, rightPWM);
    analogWrite(RIGHT_PWM2, rightPWM);
    analogWrite(LEFT_PWM1, leftPWM);
    analogWrite(LEFT_PWM2, leftPWM);
  }
  else if (msg.data == 4)
  {
  //Right
    digitalWrite(RIGHT_DIR1, LOW);
    digitalWrite(RIGHT_DIR2, LOW);
    digitalWrite(LEFT_DIR1, LOW);
    digitalWrite(LEFT_DIR2, LOW);
  //PWM Speed Control
    analogWrite(RIGHT_PWM1, rightPWM);
    analogWrite(RIGHT_PWM2, rightPWM);
    analogWrite(LEFT_PWM1, leftPWM);
    analogWrite(LEFT_PWM2, leftPWM);
  }
  else if (msg.data == 5)
  {
  //Left (45F)
    digitalWrite(RIGHT_DIR1, HIGH);
  //  digitalWrite(RIGHT_DIR2, LOW);
  //  digitalWrite(LEFT_DIR1, LOW);
    digitalWrite(LEFT_DIR2, HIGH);
  //PWM Speed Control
    analogWrite(RIGHT_PWM1, rightPWM);
//    analogWrite(RIGHT_PWM2, rightPWM);
//    analogWrite(LEFT_PWM1, leftPWM);
    analogWrite(LEFT_PWM2, leftPWM);
  }
   else if (msg.data == 6)
  {
  //Right (45F)
  //  digitalWrite(RIGHT_DIR1, LOW);
    digitalWrite(RIGHT_DIR2, LOW);
    digitalWrite(LEFT_DIR1, LOW);
  //  digitalWrite(LEFT_DIR2, LOW);
  //PWM Speed Control
//    analogWrite(RIGHT_PWM1, rightPWM);
    analogWrite(RIGHT_PWM2, rightPWM);
    analogWrite(LEFT_PWM1, leftPWM);
//    analogWrite(LEFT_PWM2, leftPWM);
  }
    else if (msg.data == 7)
  {
  //Left (45B)
  //  digitalWrite(RIGHT_DIR1, HIGH);
    digitalWrite(RIGHT_DIR2, HIGH);
    digitalWrite(LEFT_DIR1, HIGH);
  //  digitalWrite(LEFT_DIR2, HIGH);
  //PWM Speed Control
//    analogWrite(RIGHT_PWM1, rightPWM);
    analogWrite(RIGHT_PWM2, rightPWM);
    analogWrite(LEFT_PWM1, leftPWM);
//    analogWrite(LEFT_PWM2, leftPWM);
  }
    else if (msg.data == 8)
  {
  //Right (45B)
    digitalWrite(RIGHT_DIR1, LOW);
  //  digitalWrite(RIGHT_DIR2, LOW);
  //  digitalWrite(LEFT_DIR1, LOW);
    digitalWrite(LEFT_DIR2, LOW);
  //PWM Speed Control
    analogWrite(RIGHT_PWM1, rightPWM);
//    analogWrite(RIGHT_PWM2, rightPWM);
//    analogWrite(LEFT_PWM1, leftPWM);
    analogWrite(LEFT_PWM2, leftPWM);
  }
    else if (msg.data == 9)
  {
  //Rotate Left
    digitalWrite(RIGHT_DIR1, HIGH);
    digitalWrite(RIGHT_DIR2, LOW);
    digitalWrite(LEFT_DIR1, HIGH);
    digitalWrite(LEFT_DIR2, LOW);
  //PWM Speed Control
    analogWrite(RIGHT_PWM1, rightPWM);
    analogWrite(RIGHT_PWM2, rightPWM);
    analogWrite(LEFT_PWM1, leftPWM);
    analogWrite(LEFT_PWM2, leftPWM);
  }
    else if (msg.data == 10)
  {
  //Rotate Right
    digitalWrite(RIGHT_DIR1, LOW);
    digitalWrite(RIGHT_DIR2, HIGH);
    digitalWrite(LEFT_DIR1, LOW);
    digitalWrite(LEFT_DIR2, HIGH);
  //PWM Speed Control
    analogWrite(RIGHT_PWM1, rightPWM);
    analogWrite(RIGHT_PWM2, rightPWM);
    analogWrite(LEFT_PWM1, leftPWM);
    analogWrite(LEFT_PWM2, leftPWM);
  }
  else
  {
    analogWrite(RIGHT_PWM1, 0);
    analogWrite(RIGHT_PWM2, 0);
    analogWrite(LEFT_PWM1, 0);
    analogWrite(LEFT_PWM2, 0);
  }
}

//Set up the ros node (publisher and subscriber)
std_msgs::Float32 encLeft;
std_msgs::Float32 encRight;
ros::Publisher pub_encLeft("val_encLeft", &encLeft);
ros::Publisher pub_encRight("val_encRight", &encRight);

//ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", messageCb_cmd_vel);
ros::Subscriber<std_msgs::Bool> sub_reset_encLeft("/reset_encLeft", messageCb_reset_encLeft);
ros::Subscriber<std_msgs::Bool> sub_reset_encRight("/reset_encRight", messageCb_reset_encRight);
ros::Subscriber<std_msgs::Int32> sub_ax12_control("/cmd_servo_dir", messageCb_ax12_control);

//put your setup code here, to run once:
void setup()
{
//  Encoder Pins Pull-Up
  pinMode(ENCODER_COUNT_UP_RIGHT, INPUT_PULLUP);
  pinMode(ENCODER_COUNT_DOWN_RIGHT, INPUT_PULLUP);
  pinMode(ENCODER_COUNT_UP_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_COUNT_DOWN_LEFT, INPUT_PULLUP);

//  Interrupt Input
  attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_UP_RIGHT), COUNT_INTERRUPT_CW_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_DOWN_RIGHT), COUNT_INTERRUPT_CCW_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_UP_LEFT), COUNT_INTERRUPT_CW_B, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_DOWN_LEFT), COUNT_INTERRUPT_CCW_B, RISING);

//  Input/Output Pins Assigment
  pinMode(RIGHT_DIR1, OUTPUT);
  pinMode(RIGHT_PWM1, OUTPUT);
  pinMode(RIGHT_DIR2, OUTPUT);
  pinMode(RIGHT_PWM2, OUTPUT);
  pinMode(LEFT_DIR1, OUTPUT);
  pinMode(LEFT_PWM1, OUTPUT);
  pinMode(LEFT_DIR2, OUTPUT);
  pinMode(LEFT_PWM2, OUTPUT);

//  Initiate ROS-node
  nh.initNode();

  nh.advertise(pub_encLeft);
  nh.advertise(pub_encRight);

//  nh.subscribe(sub_cmd_vel);

  nh.subscribe(sub_reset_encLeft);
  nh.subscribe(sub_reset_encRight);

  nh.subscribe(sub_ax12_control);
}

//put your main code here, to run repeatedly:
void loop()
{
  encLeft.data = COUNTER_A;
  encRight.data = COUNTER_B;

  pub_encLeft.publish(&encLeft);
  pub_encRight.publish(&encRight);

  nh.spinOnce();

  delay(1);
}
