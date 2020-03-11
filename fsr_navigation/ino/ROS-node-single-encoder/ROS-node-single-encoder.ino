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
#include "std_msgs/String.h"
#include <std_msgs/Float32.h>

/* Motor Pin Assignment*/
int DIRA = 12;
int PWMA = 10;

/*Speed*/
int SPD1 = 100;

/*Encoder Pins Definition*/
#define ENCODER_COUNT_UP 3
#define ENCODER_COUNT_DOWN 2

/*Encoder Variables*/
volatile signed int TEMP, COUNTER = 0;

/*Set up the ros node*/
std_msgs::Float32 encoder_value;
ros::Publisher pub_encoder_value("encoder_value", &encoder_value);

ros::NodeHandle nh;

/*put your setup code here, to run once:*/
void setup()
{
/*	Encoder pin mode setup*/
	pinMode(ENCODER_COUNT_UP, INPUT_PULLUP);
	pinMode(ENCODER_COUNT_DOWN, INPUT_PULLUP);

/*	AttachInterrupt helper function*/
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_UP), COUNT_INTERRUPT_CW, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_DOWN), COUNT_INTERRUPT_CCW, RISING);

/*	DC motor DIR mode setup*/
	pinMode(DIRA, OUTPUT);
	pinMode(PWMA, OUTPUT);

/*	Initialize ROS node*/
	nh.initNode();

/*	ROS publisher*/
	nh.advertise(pub_encoder_value);
}

/*put your main code here, to run repeatedly:*/
void loop()
{
/*	Direction Control*/
	digitalWrite(DIRA, HIGH);
/*	PWM Speed Control*/
	analogWrite(PWMA, SPD1);
  
	encoder_value.data = COUNTER;
	pub_encoder_value.publish(&encoder_value);
	nh.spinOnce();
}

/*Counter Helper Function*/
void COUNT_INTERRUPT_CW() 
{
	if(digitalRead(ENCODER_COUNT_DOWN)==LOW) 
	{
		COUNTER++;
	}
	else
	{
		COUNTER--;
	}
}

void COUNT_INTERRUPT_CCW()
{
	if(digitalRead(ENCODER_COUNT_UP)==LOW) 
	{
		COUNTER--;
	}
	else
	{
		COUNTER++;
	}
}
