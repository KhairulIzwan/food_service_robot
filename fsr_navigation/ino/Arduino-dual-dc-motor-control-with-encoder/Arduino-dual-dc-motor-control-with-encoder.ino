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

/* Motor Pin Assignment*/
int DIRA = 12;
int PWMA = 10;
int DIRB = 13;
int PWMB = 11;

/*Speed*/
int SPD1 = 0;

/*Encoder Pins Definition*/
/*Using an interrupt pins :: 2, 3, 18, 19, 20, 21 :: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/?setlang=it*/
#define ENCODER_COUNT_UP_A 3
#define ENCODER_COUNT_DOWN_A 2
#define ENCODER_COUNT_UP_B 19
#define ENCODER_COUNT_DOWN_B 18

/*Encoder Variables*/
volatile signed int TEMP_A, COUNTER_A = 0;
volatile signed int TEMP_B, COUNTER_B = 0; 

void setup()
{
/*	Serial Communication*/
	Serial.begin (9600);

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
}

void loop()
{
/*	Direction Control*/
	digitalWrite(DIRA, HIGH);
	digitalWrite(DIRB, HIGH);
/*	PWM Speed Control*/
	analogWrite(PWMA, SPD1);
	analogWrite(PWMB, SPD1);

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
