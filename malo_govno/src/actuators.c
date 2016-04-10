#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "system.h"
#include "can.h"
#include "actuators.h"

void servo_init(unsigned int f_pwm)
{
	DDRE |= ((1 << PINE3) | (1 << PINE4) | (1 << PINE5));
	
	TCNT3 = 0;
	
	OCR3A = 0;
	OCR3B = 0;
	OCR3C = 0;
	
	TCCR3A = (1 << COM3A1)  | (1 << COM3B1) | (1 << COM3B0) | (1 << COM3C1) | (1 << COM3C0) | (1 << WGM31);
	TCCR3B = (1<< CS31) | (1 << WGM32) | (1 << WGM33) ; // PRESKALER = 8
	
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << COM1B1) | (1 << COM1C1) | (1 << COM1C0) | (1 << WGM11);
	TCCR1B = (1 << CS11) | (1 << WGM12) | (1 << WGM13);
	
	ICR3   = ((double)F_CPU) / (8.0 * f_pwm) + 0.5;
	ICR1   = ((double)F_CPU) / (8.0 * f_pwm) + 0.5;
}//END OF servo_init


static void servo_set_duty_cycle_umbrella(int16_t value)
{
	uint16_t temp = ((double)ICR3 / 255.0) * value + 0.5;
	OCR3AH = temp >> 8;
	OCR3AL = temp & 0xFF;
}
static void servo_set_duty_cycle_door(int16_t value)
{
	uint16_t temp = ((double)ICR3 / 255.0) * value + 0.5;
	OCR3BH = temp >> 8;
	OCR3BL = temp & 0xFF;
}
static void servo_set_duty_cycle_left_grabber(int16_t value)
{
	uint16_t temp = ((double)ICR3 / 255.0) * value + 0.5;
	OCR3CH = temp >> 8;
	OCR3CL = temp & 0xFF;
}
static void servo_set_duty_cycle_right_grabber(int16_t value)
{
	uint16_t temp = ((double)ICR3 / 255.0) * value + 0.5;
	OCR1AH = temp >> 8;
	OCR1AL = temp & 0xFF;
}

//////////////////////////////////////////////////////////////////////////

void servo_set_umbrella_position(int8_t angle)//90 je otvoreno -86 zatvoreno 
{
	servo_set_duty_cycle_umbrella( 255-(236.0 - ((double)angle / 90.0) * 11.4));
}
void servo_set_door_position(int8_t angle)
{
	servo_set_duty_cycle_door( 255-(236.0 - ((double)angle / 90.0) * 11.4));
}
void servo_set_left_grabber_position(int8_t angle)
{
	servo_set_duty_cycle_left_grabber( 255-(236.0 - ((double)angle / 90.0) * 11.4));
}
void servo_set_right_grabber_position(int8_t angle)
{
	servo_set_duty_cycle_right_grabber( 255-(236.0 - ((double)angle / 90.0) * 11.4));
}
void servo_set_grabbers(int state)
{
	if(UP)
	{
		/*
		servo_set_left_grabber_position();//find
		servo_set_right_grabber_position();//find
		*/
	}
	else
	{
		/*
		servo_set_left_grabber_position();//find
		servo_set_right_grabber_position();//find
		*/
	}
}
void actuators_umbrella(void)
{
	servo_set_umbrella_position(-86);
	_delay_ms(1000);
	servo_set_door_position(0);
	_delay_ms(1000);
}
void actuators_setup(void)
{
	servo_set_door_position(90);
	_delay_ms(1000);
	servo_set_umbrella_position(85);
	_delay_ms(1000);
	servo_set_door_position(0);
	_delay_ms(500);
	servo_set_grabbers(UP);
}
