/*
 * SPI_DRIVERS_config.h
 *
 * Created: 12/5/2018 6:40:16 PM
 *  Author: Alzahraa Elsallakh
 */ 


#ifndef SPI_DRIVERS_CONFIG_H_
#define SPI_DRIVERS_CONFIG_H_

#endif /* SPI_DRIVERS_CONFIG_H_ */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#define PI 3.14159265358979323846

#define L 0.42
#define R  0.06

#define maxVolt 6.3
#define maxLinearVelocity 0.69
#define maxVelocity 0.797
#define maxW (PI/3.0)

#define F_CPU 16000000UL


// Master In Slave Output Configuration 
#define MISO (1<<PB3)

// Motors Forward & Backward
#define MOTOR_RIGHT_F (1<<PE3)
#define MOTOR_RIGHT_B (1<<PE4)

#define MOTOR_LEFT_F (1<<PE5)
#define MOTOR_LEFT_B (1<<PE6)


// PWM as Output
#define MOTOR1_PWM (1<<PB5)
#define MOTOR2_PWM (1<<PB7)
#define MOTOR3_PWM (1<<PB4)
#define MOTOR4_PWM (1<<PB6)

