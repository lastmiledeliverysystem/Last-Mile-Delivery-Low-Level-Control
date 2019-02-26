/*
 * SPI_DRIVERS.c
 *
 * Created: 12/5/2018 6:38:29 PM
 * Author : Alzahraa Elsallakh
 */ 

#include "SPI_DRIVERS_config.h"

//Global v & w
float v,w; // m/s, rad/s
float preiousV, previousW;

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
	float rightVelocity ,leftVelocity; // rpm
	float vR, vL; // m/s

	if (v == 0.0 && w == 0.0) //Stop Motors
	{
		rightVelocity = leftVelocity = 0.0;
	}
	else
	{	
		if (v == 0) // Pure Angular Velocity
		{
			// out of allowable boundaries
			if ( fabsf(w) < minAngularVelocity || fabsf(w) > maxAngularVelocity)
			{
				// apply previous action
				v = preiousV;
				w = previousW;
			}
		}
		else if (w == 0) // Pure Translation
		{
			// out of allowable boundaries
			if ( fabsf(v) < minWheelVelocity || fabsf(v) > maxWheelVelocity)
			{
				// apply previous action
				v = preiousV;
				w = previousW;
			}
		}

		vR = (v + (w * L /2.0));  
		vL = (v - (w * L /2.0));

		if (vR == 0) // taking curve with the only left wheels 
		{
			// out of allowable boundaries
			if (fabsf(vL) < minWheelVelocity || fabsf(vL) > maxWheelVelocity)
			{
				//apply previous action
				v = preiousV;
				w = previousW;
			}
		}
		else if (vL == 0) // taking curve with the only right wheels 
		{
			// out of allowable boundaries
			if (fabsf(vR) < minWheelVelocity || fabsf(vR) > maxWheelVelocity)
			{
				//apply previous action
				v = preiousV;
				w = previousW;
			}
		}
		else // regular movement
		{
			if (fabsf(vL) < minWheelVelocity || fabsf(vR) > maxWheelVelocity)
			{
				//apply previous action
				v = preiousV;
				w = previousW;	
			}
		}


		// Ready to be applied (rpm)
		rightVelocity = (v + (w * L /2.0)) * (30.0 / (PI * R) );  
		leftVelocity = (v - (w * L /2.0)) * (30.0 /  (PI * R) );

		// Saving State
		preiousV = v;
		previousW = w;
		
		if (rightVelocity < 0.0)
		{
			 forward_right = 0;
		}
		else 
		{
			forward_right = 1;
		}

		if (leftVelocity < 0.0)
		{
			forward_left = 0;
		}
		else
		{
			forward_left = 1;
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
	
	preiousV = 0.0;
	previousW 0.0;
	
    while (1)  
    {

    }
}