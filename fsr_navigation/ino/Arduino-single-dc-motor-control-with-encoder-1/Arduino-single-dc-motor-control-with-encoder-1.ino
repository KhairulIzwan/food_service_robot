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
int DIRA = 4;
int PWMA = 5;

/*Speed*/
int SPD1 = 0;

/*Encoder Pins Definition*/
#define ENCODER_COUNT_UP 2
#define ENCODER_COUNT_DOWN 3

/*Encoder Variables*/
volatile signed int TEMP, COUNTER = 0; 

void setup()
{
/*	Serial Communication*/
	Serial.begin (9600);

/*	Encoder Pins Pull-Up*/
	pinMode(ENCODER_COUNT_UP, INPUT_PULLUP);
	pinMode(ENCODER_COUNT_DOWN, INPUT_PULLUP);

/*	Interrupt Input*/
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_UP), COUNT_INTERRUPT_CW, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_DOWN), COUNT_INTERRUPT_CCW, RISING);

/*	Input/Output Pins Assigment*/
	pinMode(DIRA, OUTPUT);
	pinMode(PWMA, OUTPUT);
}

void loop()
{
/*	Direction Control*/
	digitalWrite(DIRA, HIGH);
/*	PWM Speed Control*/
	analogWrite(PWMA, SPD1);

/*	Send the value of counter*/
	Serial.print ("COUNTER");
	Serial.print (":");
	Serial.print (COUNTER);
	Serial.println();
}

/*Counter Helper Function*/
/*Count Up*/
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

/*Count Down*/
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
