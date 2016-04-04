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
<<<<<<< HEAD
<<<<<<< HEAD
	ICR3   = ((double)F_CPU) / (8.0 * f_pwm) + 0.5;
}//END OF servo_init


static void servo_set_duty_cycle_kisobran(int16_t value)
=======
	ICR3   = ((double)F_CPU) / (8.0 * f_pwm) + 0.5; 
}//END OF servo_init


static void servo_set_duty_cycle(int16_t value)
>>>>>>> e0b240b1c14c4966c5f4aab6a35058ead1d19ffe
{
	uint16_t temp = ((double)ICR3 / 255.0) * value + 0.5;
	OCR3AH = temp >> 8;
	OCR3AL = temp & 0xFF;
	
}//END OF servo_position

<<<<<<< HEAD
void servo_set_kisobran_position(int8_t angle)//90 je otvoreno -86 zatvoreno 
{
	servo_set_duty_cycle_kisobran( 255-(236.0 - ((double)angle / 90.0) * 11.4));

}

static void servo_set_duty_cycle_vrata(int16_t value)
{
	uint16_t temp = ((double)ICR3 / 255.0) * value + 0.5;
	OCR3BH = temp >> 8;
	OCR3BL = temp & 0xFF;
	
}//END OF servo_position
void servo_set_vrata_position(int8_t angle)
{
	servo_set_duty_cycle_vrata( 255-(236.0 - ((double)angle / 90.0) * 11.4));
}
void actuators_setup_kisobran(void)
{
	servo_set_kisobran_position(-86);
	_delay_ms(1000);
	servo_set_vrata_position(0);
	_delay_ms(1000);
}
void actuators_kisobran(void)
{
	servo_set_vrata_position(90);
	_delay_ms(1000);
	servo_set_kisobran_position(85);
	_delay_ms(1000);
	servo_set_vrata_position(0);
}
=======
=======
	ICR3   = ((double)F_CPU) / (8.0 * f_pwm) + 0.5; 
}//END OF servo_init


static void servo_set_duty_cycle(int16_t value)
{
	uint16_t temp = ((double)ICR3 / 255.0) * value + 0.5;
	OCR3AH = temp >> 8;
	OCR3AL = temp & 0xFF;
	
}//END OF servo_position

>>>>>>> 07363ff7400dbe0af1895fa548283af7a30f1eb3
void servo_set_position(int8_t angle)
{
	servo_set_duty_cycle( 255-(236.0 - ((double)angle / 90.0) * 11.4));

}
<<<<<<< HEAD
>>>>>>> e0b240b1c14c4966c5f4aab6a35058ead1d19ffe
=======
>>>>>>> 07363ff7400dbe0af1895fa548283af7a30f1eb3
