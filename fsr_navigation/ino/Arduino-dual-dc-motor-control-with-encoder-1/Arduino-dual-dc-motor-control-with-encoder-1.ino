/*
 * Title: Dual DC Motor Control with Encoder
 * Author: Khairul Izwan 22-03-2020
 * Description: Controlling Dual DC Motor with Encoder using 10Amp 7V-30V DC Motor Driver Shield for Arduino (2 Channels)
 */

/* Parts ::
 * 1. DC Motor:: 12V 38RPM 5kgfcm Brushed DC Geared Motor :: https://my.cytron.io/p-12v-38rpm-5kgfcm-brushed-dc-geared-motor?search=12%2038rpm&description=1&src=search.list
 * 2. Driver:: Shield L298P Motor Driver with GPIO :: https://my.cytron.io/p-shield-l298p-motor-driver-with-gpio?src=search.list
 */

//Motor Pin Assignment
int DIRA = 4;
int PWMA = 5;
int DIRB = 12;
int PWMB = 10;

//Speed
int SPDA = 0;
int SPDB = 0;

//Encoder Pins Definition
//Using an interrupt pins :: 2, 3, 18, 19, 20, 21 :: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/?setlang=it
#define ENCODER_COUNT_UP_A 3
#define ENCODER_COUNT_DOWN_A 2
#define ENCODER_COUNT_UP_B 21
#define ENCODER_COUNT_DOWN_B 20

//Encoder Variables
volatile signed int TEMP_A, COUNTER_A = 0;
volatile signed int TEMP_B, COUNTER_B = 0; 

//Counter Helper Function
//Count Up A
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

//Count Down A
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

//Count Up B
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

//Count Down A
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

void setup()
{
//  Serial Communication
	Serial.begin (9600);

//  Encoder Pins Pull-Up
	pinMode(ENCODER_COUNT_UP_A, INPUT_PULLUP);
	pinMode(ENCODER_COUNT_DOWN_A, INPUT_PULLUP);
	pinMode(ENCODER_COUNT_UP_B, INPUT_PULLUP);
	pinMode(ENCODER_COUNT_DOWN_B, INPUT_PULLUP);

//  Interrupt Input
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_UP_A), COUNT_INTERRUPT_CW_A, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_DOWN_A), COUNT_INTERRUPT_CCW_A, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_UP_B), COUNT_INTERRUPT_CW_B, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_DOWN_B), COUNT_INTERRUPT_CCW_B, RISING);

//Input/Output Pins Assigment
	pinMode(DIRA, OUTPUT);
	pinMode(PWMA, OUTPUT);
	pinMode(DIRB, OUTPUT);
	pinMode(PWMB, OUTPUT);
}

void loop()
{
//  Direction Control
	digitalWrite(DIRA, HIGH);
	digitalWrite(DIRB, HIGH);
	
//	PWM Speed Control
	analogWrite(PWMA, SPDA);
	analogWrite(PWMB, SPDB);
	
//	Send the value of counter
	Serial.print ("COUNTER_A");
	Serial.print (":");
	Serial.print (COUNTER_A);
	Serial.print ("\t");
	Serial.print ("COUNTER_B");
	Serial.print (":");
	Serial.print (COUNTER_B);
	Serial.println();
}
