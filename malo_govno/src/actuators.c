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
	ICR3   = ((double)F_CPU) / (8.0 * f_pwm) + 0.5; 
}//END OF servo_init


static void servo_set_duty_cycle(int16_t value)
{
	uint16_t temp = ((double)ICR3 / 255.0) * value + 0.5;
	OCR3AH = temp >> 8;
	OCR3AL = temp & 0xFF;
	
}//END OF servo_position

void servo_set_position(int8_t angle)
{
	servo_set_duty_cycle( 255-(236.0 - ((double)angle / 90.0) * 11.4));

}