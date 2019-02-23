/*
 * SPI_DRIVERS.c
 *
 * Created: 12/5/2018 6:38:29 PM
 * Author : Alzahraa Elsallakh
 */ 

#include "SPI_DRIVERS_config.h"

//Global v & w
float v,w;

//Interrupts Variables
float msg;
float received [4];
int counter = 0;

int forward_right = 0;
int forward_left = 0;


void timer0_init()
{
	TCCR0 |= (1<<WGM00)|(1<<WGM01)|(1<<COM01)|(1<<CS00) ;	
}
void timer1_init()
{
	TCCR1A |= (1<<WGM10)|(1<<COM1A1)|(1<<COM1B1);
	TCCR1B |= (1<<WGM12)|(1<<CS10);
}
void timer2_init()
{
	TCCR2 |= (1<<WGM20)|(1<<WGM21)|(1<<COM21)|(1<<CS20) ;
}

void spi_init_slave()
{
	DDRB |= MISO;
	SPCR = (1<<SPE) | (1<<SPIE);
}

void enabling()
{
	PORTE |=  MOTOR_RIGHT_F ;
	PORTE |=  MOTOR_RIGHT_B ;
	PORTE |= MOTOR_LEFT_F ;
	PORTE |= MOTOR_LEFT_B ;
}

ISR (SPI_STC_vect)
{
	//counter == 0 >> value(v)
	//counter == 1 >> sign(v), if 1 >> negative
	//counter == 2 >> value(w)
	//counter == 3 >> sign(w), if 1 >> negative
	
	msg = (float)SPDR;
	
	received [counter] = msg;
	counter++;
	
	if (counter == 4)
	{
		counter = 0;
		
		v = received [0];
		if  (received [1] == 1)
		{
			v = -1 * v;
		}
		
		w = received[2];
		w = w * PI / 180.0;
		if  (received [3] == 1)
		{
			w = -1 * w;
		}
		
		calculate_velocities(v,w);
	}	
}

void calculate_pwms (float voltR, float voltL)
{	
	int pwmR, pwmL;
	pwmR = (int)((voltR /  maxVolt) * 255);

	pwmL = (int)((voltL /  maxVolt) * 255);
	
	if (forward_left)
	{
		OCR0 =0;	
		OCR1A = pwmL;
	}
	else 
	{
		OCR0 = pwmL;
		OCR1A = 0;
	}
	
	if (forward_right)
	{
		OCR2 =0;
		OCR1B = pwmR;
	}
	else
	{
		OCR2 = pwmR;
		OCR1B = 0;
	}
	
}

void calculate_voltages(float velocityR,float velocityL)
{
	float voltR,voltL;
	
	//rpm mappting to volt
	voltL = (0.04175 * velocityR) + 0.59855;
	voltR = (0.04205 * velocityL) + 0.6013;
	
	if ( velocityR == 0.0 )
	{
		voltR = 0.0;
	}
	if ( velocityL == 0.0 )
	{
		voltL = 0.0;
	}
	
	calculate_pwms(voltR, voltL);
}

void calculate_velocities(float v,float w)
{
	float rightVelocity ,leftVelocity;
	
	//Stop Motors
	if (v == 0.0 && w == 0.0)
	{
		rightVelocity = leftVelocity = 0.0;
	}
	else
	{	
		//threshold
		if (v > maxLinearVelocity)
		{
			v = maxLinearVelocity;
		}
		else if (v < (-1 * maxLinearVelocity))
		{
			v = -1 * maxLinearVelocity;
		}
	
		if ( w > maxW)
		{
			w = maxW;
		}
		else if ( w < (-1 * maxW))
		{
			w = -1 * maxW;
		}
		
		// v : m/s, w : rad/s 
		// rpm = m/s * (30 / PI * R)
		rightVelocity = (v + (w * L /2.0)) * (30.0 / (PI * R)); //rpm
		leftVelocity = (v - (w * L /2.0)) * (30.0 / (PI * R)); //rpm 
	
		if (rightVelocity <= 0.0)
		{
			 forward_right = 0;
		}
		else if (rightVelocity > 0.0)
		{
			forward_right = 1;
		}
		if (leftVelocity <= 0.0)
		{
			forward_left = 0;
		}
		else if (leftVelocity > 0.0)
		{
			forward_left = 1;
		}
		
		//threshold
		if (rightVelocity > maxVelocity)
		{
			rightVelocity = maxVelocity;
		}
		else if (rightVelocity < (-1 * maxVelocity)) 
		{
			rightVelocity = -1 * maxVelocity;
		}
		if (leftVelocity > maxVelocity) 
		{
			leftVelocity = maxVelocity;
		}
		if (leftVelocity < (-1 * maxVelocity)) 
		{
			leftVelocity = -1 * maxVelocity;
		}
		
	}
	
	calculate_voltages(fabsf(rightVelocity),fabsf(leftVelocity));
}

int main(void)
{	
	//Motor directions (Forward & Backward)
	DDRE |= MOTOR_RIGHT_F | MOTOR_RIGHT_B | MOTOR_LEFT_F | MOTOR_LEFT_B ;
	
	//PWM Configuration
	DDRB |= MOTOR1_PWM | MOTOR2_PWM | MOTOR3_PWM | MOTOR4_PWM;
	
	//Global Interrupt Enable
	sei();
	
	//Timers Initiation
	timer0_init();
	timer1_init();
	timer2_init();
	
	//Enable pins
	enabling();

	//SPI Initiation
	spi_init_slave();
	
	
    while (1)  
    {

    }
}