/*
 * SPI_DRIVERS.h
 *
 * Created: 12/5/2018 6:40:36 PM
 *  Author: Alzahraa Elsallakh
 */ 


#ifndef SPI_DRIVERS_H_
#define SPI_DRIVERS_H_





#endif /* SPI_DRIVERS_H_ */

void timer0_init();
void timer1_init();
void timer2_init();

void spi_init_slave();

void enabling();

void calculate_pwms (float,float, float, float);
void calculate_voltages(float,float);
void calculate_velocities(float,float);
