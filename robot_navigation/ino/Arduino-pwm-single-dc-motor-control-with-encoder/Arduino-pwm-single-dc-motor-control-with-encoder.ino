/*Title: DC Motor Control with Encoder (Single)*/
/*Author: Khairul Izwan 10032020_*/
/*Description: Controlling DC Motor with Encoder*/

/* Motor Pin Assignment*/
int DIR1 = 4;
int PWM1 = 5;

/*Speed*/
int SPD1 = 0;

/*Encoder Pins Definition*/
#define ENCODER_COUNT_UP 3
#define ENCODER_COUNT_DOWN 2

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
	pinMode(DIR1, OUTPUT);
}

void loop()
{
/*	Direction Control*/
	digitalWrite(DIR1,HIGH);
/*	PWM Speed Control*/
	analogWrite(PWM1, SPD1);

/*	Send the value of counter*/
	Serial.print ("COUNTER");
	Serial.print (":");
	Serial.print (COUNTER);
	Serial.println();
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
