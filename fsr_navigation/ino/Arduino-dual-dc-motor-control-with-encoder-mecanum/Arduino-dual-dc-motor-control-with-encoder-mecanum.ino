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

void setup()
{
//	Serial Communication
	Serial.begin (9600);

//	Encoder Pins Pull-Up
	pinMode(ENCODER_COUNT_UP_RIGHT, INPUT_PULLUP);
	pinMode(ENCODER_COUNT_DOWN_RIGHT, INPUT_PULLUP);
	pinMode(ENCODER_COUNT_UP_LEFT, INPUT_PULLUP);
	pinMode(ENCODER_COUNT_DOWN_LEFT, INPUT_PULLUP);

//	Interrupt Input
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_UP_RIGHT), COUNT_INTERRUPT_CW_A, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_DOWN_RIGHT), COUNT_INTERRUPT_CCW_A, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_UP_LEFT), COUNT_INTERRUPT_CW_B, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_DOWN_LEFT), COUNT_INTERRUPT_CCW_B, RISING);

//	Input/Output Pins Assigment
	pinMode(RIGHT_DIR1, OUTPUT);
	pinMode(RIGHT_PWM1, OUTPUT);
	pinMode(RIGHT_DIR2, OUTPUT);
	pinMode(RIGHT_PWM2, OUTPUT);
  pinMode(LEFT_DIR1, OUTPUT);
  pinMode(LEFT_PWM1, OUTPUT);
  pinMode(LEFT_DIR2, OUTPUT);
  pinMode(LEFT_PWM2, OUTPUT);
}

void loop()
{
//	Direction Control

////Forward
//	digitalWrite(RIGHT_DIR1, HIGH);
//	digitalWrite(RIGHT_DIR2, LOW);
//  digitalWrite(LEFT_DIR1, LOW);
//  digitalWrite(LEFT_DIR2, HIGH);
////PWM Speed Control
//  analogWrite(RIGHT_PWM1, SPD1);
//  analogWrite(RIGHT_PWM2, SPD1);
//  analogWrite(LEFT_PWM1, SPD1);
//  analogWrite(LEFT_PWM2, SPD1);

////Backward
//  digitalWrite(RIGHT_DIR1, LOW);
//  digitalWrite(RIGHT_DIR2, HIGH);
//  digitalWrite(LEFT_DIR1, HIGH);
//  digitalWrite(LEFT_DIR2, LOW);
////PWM Speed Control
//  analogWrite(RIGHT_PWM1, SPD1);
//  analogWrite(RIGHT_PWM2, SPD1);
//  analogWrite(LEFT_PWM1, SPD1);
//  analogWrite(LEFT_PWM2, SPD1);

////Right
//  digitalWrite(RIGHT_DIR1, LOW);
//  digitalWrite(RIGHT_DIR2, LOW);
//  digitalWrite(LEFT_DIR1, LOW);
//  digitalWrite(LEFT_DIR2, LOW);
////PWM Speed Control
//  analogWrite(RIGHT_PWM1, SPD1);
//  analogWrite(RIGHT_PWM2, SPD1);
//  analogWrite(LEFT_PWM1, SPD1);
//  analogWrite(LEFT_PWM2, SPD1);

////Left
//  digitalWrite(RIGHT_DIR1, HIGH);
//  digitalWrite(RIGHT_DIR2, HIGH);
//  digitalWrite(LEFT_DIR1, HIGH);
//  digitalWrite(LEFT_DIR2, HIGH);
////PWM Speed Control
//  analogWrite(RIGHT_PWM1, SPD1);
//  analogWrite(RIGHT_PWM2, SPD1);
//  analogWrite(LEFT_PWM1, SPD1);
//  analogWrite(LEFT_PWM2, SPD1);

////Right (45F)
////  digitalWrite(RIGHT_DIR1, LOW);
//  digitalWrite(RIGHT_DIR2, LOW);
//  digitalWrite(LEFT_DIR1, LOW);
////  digitalWrite(LEFT_DIR2, LOW);
////PWM Speed Control
////	analogWrite(RIGHT_PWM1, SPD1);
//  analogWrite(RIGHT_PWM2, SPD1);
//  analogWrite(LEFT_PWM1, SPD1);
////  analogWrite(LEFT_PWM2, SPD1);

////Right (45B)
//  digitalWrite(RIGHT_DIR1, LOW);
////  digitalWrite(RIGHT_DIR2, LOW);
////  digitalWrite(LEFT_DIR1, LOW);
//  digitalWrite(LEFT_DIR2, LOW);
////PWM Speed Control
//  analogWrite(RIGHT_PWM1, SPD1);
////  analogWrite(RIGHT_PWM2, SPD1);
////  analogWrite(LEFT_PWM1, SPD1);
//  analogWrite(LEFT_PWM2, SPD1);

////Left (45F)
//  digitalWrite(RIGHT_DIR1, HIGH);
////  digitalWrite(RIGHT_DIR2, LOW);
////  digitalWrite(LEFT_DIR1, LOW);
//  digitalWrite(LEFT_DIR2, HIGH);
////PWM Speed Control
//  analogWrite(RIGHT_PWM1, SPD1);
////  analogWrite(RIGHT_PWM2, SPD1);
////  analogWrite(LEFT_PWM1, SPD1);
//  analogWrite(LEFT_PWM2, SPD1);

////Left (45B)
////  digitalWrite(RIGHT_DIR1, HIGH);
//  digitalWrite(RIGHT_DIR2, HIGH);
//  digitalWrite(LEFT_DIR1, HIGH);
////  digitalWrite(LEFT_DIR2, HIGH);
////PWM Speed Control
////  analogWrite(RIGHT_PWM1, SPD1);
//  analogWrite(RIGHT_PWM2, SPD1);
//  analogWrite(LEFT_PWM1, SPD1);
////  analogWrite(LEFT_PWM2, SPD1);

////Rotate Right
//  digitalWrite(RIGHT_DIR1, LOW);
//  digitalWrite(RIGHT_DIR2, HIGH);
//  digitalWrite(LEFT_DIR1, LOW);
//  digitalWrite(LEFT_DIR2, HIGH);
////PWM Speed Control
//  analogWrite(RIGHT_PWM1, SPD1);
//  analogWrite(RIGHT_PWM2, SPD1);
//  analogWrite(LEFT_PWM1, SPD1);
//  analogWrite(LEFT_PWM2, SPD1);

//Rotate Left
  digitalWrite(RIGHT_DIR1, HIGH);
  digitalWrite(RIGHT_DIR2, LOW);
  digitalWrite(LEFT_DIR1, HIGH);
  digitalWrite(LEFT_DIR2, LOW);
//PWM Speed Control
  analogWrite(RIGHT_PWM1, SPD1);
  analogWrite(RIGHT_PWM2, SPD1);
  analogWrite(LEFT_PWM1, SPD1);
  analogWrite(LEFT_PWM2, SPD1);

/*	Send the value of counter*/
	Serial.print ("COUNTER_A");
	Serial.print (":");
	Serial.print (COUNTER_A);
	Serial.print ("\t");
	Serial.print ("COUNTER_B");
	Serial.print (":");
	Serial.print (COUNTER_B);
	Serial.println();
}
