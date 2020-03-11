/*Title: DC Motor Control with Encoder (Single)*/
/*Author: Khairul Izwan 10-03-2020*/
/*Description: Controlling DC Motor with Encoder using */

/*Parts*/
/*1. DC Motor:: 12V 38RPM 5kgfcm Brushed DC Geared Motor with Encoder :: https://my.cytron.io/p-12v-38rpm-5kgfcm-brushed-dc-geared-motor-with-encoder?search=12v%2038RPM&description=1&src=search.list */
/*2. Driver:: Shield L298P Motor Driver with GPIO :: https://my.cytron.io/p-shield-l298p-motor-driver-with-gpio?src=search.list*/

/*Notes:*/
/*D10, D11, D12 and D13 for motor control*/
/*D10 controls speed of motor A and D11 controls speed of motor B.*/
/*Pin D12 controls direction of motor A and Pin D13 controls direction of motor B.*/

/*include necessary library*/
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

/* Motor Pin Assignment*/
int DIRA = 12;
int PWMA = 10;
int DIRB = 13;
int PWMB = 11;

/*Min Speed for the motor*/
int SPD1 = 50;

/*Encoder Pins Definition*/
/*Using an interrupt pins :: 2, 3, 18, 19, 20, 21 :: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/?setlang=it*/
#define ENCODER_COUNT_UP_A 3
#define ENCODER_COUNT_DOWN_A 2
#define ENCODER_COUNT_UP_B 19
#define ENCODER_COUNT_DOWN_B 18

/*Encoder Variables*/
volatile signed int TEMP_A, COUNTER_A = 0;
volatile signed int TEMP_B, COUNTER_B = 0; 

/*Change according to the robot wheel dimension*/
#define wheelSep 0.14 // mm
#define wheelRadius 0.05; // mm

/*Variables declaration*/
float transVelocity;
float rotVelocity;
float velDiff;

float leftPWM;
float rightPWM;

float leftPower;
float rightPower;

/*Callback function for geometry_msgs::Twist */
void messageCb_cmd_vel(const geometry_msgs::Twist &msg)
{
/*	Get the ros topic value*/
	transVelocity = msg.linear.x;
	rotVelocity = msg.angular.z;

/*	Differential Drive Kinematics::http://www.cs.columbia.edu/~allen/F15/NOTES/icckinematics.pdf*/
	velDiff = (wheelSep * rotVelocity) / 2.0;
	leftPower = (transVelocity + velDiff) / wheelRadius;
	rightPower = (transVelocity - velDiff) / wheelRadius;

/*	Normalize the rad/sec --> analog value 0 ~ 255 (arduino)*/
	leftPWM = (leftPower - 0) * (255 - 0) / (4.4 - 0) + 0;
	rightPWM = (rightPower - 0) * (255 - 0) / (4.4 - 0) + 0;

/*	motor directection helper function*/
	motorDirection();
}

/*Motor Direction helper function*/
void motorDirection()
{
	if (leftPWM > 0 and rightPWM > 0)
	{
		digitalWrite(DIRA, LOW);
		digitalWrite(DIRB, LOW);
		analogWrite(PWMA, max(abs(leftPWM), SPD1));
		analogWrite(PWMB, max(abs(rightPWM), SPD1));
	}
	else if (leftPWM < 0 and rightPWM < 0)
	{
		digitalWrite(DIRA, HIGH);
		digitalWrite(DIRB, HIGH);
		analogWrite(PWMA, max(abs(leftPWM), SPD1));
		analogWrite(PWMB, max(abs(rightPWM), SPD1));
	}
	else if (leftPWM < 0 and rightPWM > 0)
	{
		digitalWrite(DIRA, LOW);
		digitalWrite(DIRB, HIGH);
		analogWrite(PWMA, max(abs(leftPWM), SPD1));
		analogWrite(PWMB, max(abs(rightPWM), SPD1));
	}
	else if (leftPWM > 0 and rightPWM < 0)
	{
		digitalWrite(DIRA, HIGH);
		digitalWrite(DIRB, LOW);
		analogWrite(PWMA, max(abs(leftPWM), SPD1));
		analogWrite(PWMB, max(abs(rightPWM), SPD1));
	}
	else if (leftPWM == 0 and rightPWM == 0)
	{
		analogWrite(PWMA, 0);
		analogWrite(PWMB, 0);
	}
}

/*Counter Helper Function*/
/*Count Up A*/
void COUNT_INTERRUPT_CW_A() 
{
	if(digitalRead(ENCODER_COUNT_DOWN_A)==LOW) 
	{
		COUNTER_A++;
	}
	else
	{
		COUNTER_A--;
	}
}

/*Count Down A*/
void COUNT_INTERRUPT_CCW_A()
{
	if(digitalRead(ENCODER_COUNT_UP_A)==LOW) 
	{
		COUNTER_A--;
	}
	else
	{
		COUNTER_A++;
	}
}

/*Count Up B*/
void COUNT_INTERRUPT_CW_B() 
{
	if(digitalRead(ENCODER_COUNT_DOWN_B)==LOW) 
	{
		COUNTER_B++;
	}
	else
	{
		COUNTER_B--;
	}
}

/*Count Down A*/
void COUNT_INTERRUPT_CCW_B()
{
	if(digitalRead(ENCODER_COUNT_UP_B)==LOW) 
	{
		COUNTER_B--;
	}
	else
	{
		COUNTER_B++;
	}
}

void messageCb_reset_encoder_value_a(const std_msgs::Bool &msg)
{
	if (msg.data == true)
	{
		COUNTER_A = 0;
	}
}

void messageCb_reset_encoder_value_b(const std_msgs::Bool &msg)
{
	if (msg.data == true)
	{
		COUNTER_B = 0;
	}
}

/*Set up the ros node*/
std_msgs::Float32 encoder_value_a;
ros::Publisher pub_encoder_value_a("encoder_value_a", &encoder_value_a);

std_msgs::Float32 encoder_value_b;
ros::Publisher pub_encoder_value_b("encoder_value_b", &encoder_value_b);

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", messageCb_cmd_vel);
ros::Subscriber<std_msgs::Bool> sub_reset_encoder_value_a("/reset_encoder_value_a", messageCb_reset_encoder_value_a);
ros::Subscriber<std_msgs::Bool> sub_reset_encoder_value_b("/reset_encoder_value_b", messageCb_reset_encoder_value_b);

ros::NodeHandle nh;

//put your setup code here, to run once:
void setup()
{
/*	Encoder Pins Pull-Up*/
	pinMode(ENCODER_COUNT_UP_A, INPUT_PULLUP);
	pinMode(ENCODER_COUNT_DOWN_A, INPUT_PULLUP);
	pinMode(ENCODER_COUNT_UP_B, INPUT_PULLUP);
	pinMode(ENCODER_COUNT_DOWN_B, INPUT_PULLUP);

/*	Interrupt Input*/
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_UP_A), COUNT_INTERRUPT_CW_A, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_DOWN_A), COUNT_INTERRUPT_CCW_A, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_UP_B), COUNT_INTERRUPT_CW_B, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_DOWN_B), COUNT_INTERRUPT_CCW_B, RISING);

/*	Input/Output Pins Assigment*/
	pinMode(DIRA, OUTPUT);
	pinMode(PWMA, OUTPUT);
	pinMode(DIRB, OUTPUT);
	pinMode(PWMB, OUTPUT);

/*	Initialize ROS node*/
	nh.initNode();

/*	ROS publisher*/
	nh.advertise(pub_encoder_value_a);
	nh.advertise(pub_encoder_value_b);

/*	ROS subscriber*/
	nh.subscribe(sub);
	nh.subscribe(sub_reset_encoder_value_a);
	nh.subscribe(sub_reset_encoder_value_b);
}

//put your main code here, to run repeatedly:
void loop()
{
	encoder_value_a.data = COUNTER_A;
	encoder_value_b.data = COUNTER_B;

	pub_encoder_value_a.publish(&encoder_value_a);
	pub_encoder_value_b.publish(&encoder_value_b);

	nh.spinOnce();
}
