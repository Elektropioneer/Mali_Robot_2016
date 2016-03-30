#include <stdint.h>
#include "system.h"
#include "can.h"
#include "gpio.h"
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "actuators.h"
#include "list_generic.h"

static volatile unsigned long sys_time;
static void (*timer_callback)(void) = NULL;

unsigned int received = 0;

void timer_register_callback(void (*callback)(void))
{
    timer_callback = callback;
}


void timer_init(unsigned int freq)
{
    TCCR1A = 0;
	TCCR1B = (1 << WGM12) | (1 << CS10);
	OCR1A = (double)F_CPU / (double)freq + 0.5;
	TIMSK1 = 1 << OCIE1A;

	SREG |= 0x80;
}


ISR(TIMER1_COMPA_vect)
{
    if(timer_callback != NULL)
        timer_callback();
	sys_time++;
}

void system_init(void)
{
	_delay_ms(1000);
	DDRG = 0xff;
	PORTG = 0xFF;
	_delay_ms(1000);
	PORTG = 0x00;
	servo_init(50);
	timer_init(1000);
	CAN_Init(1);
}
uint32_t system_get_system_time(void)
{
	return sys_time;
}

void system_reset_system_time(void)
{
    sys_time = 0;
}
