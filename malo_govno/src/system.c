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
static uint8_t match_started;
static void (*timer_callback)(void) = NULL;
static int combination[2];

unsigned int received = 0;

void timer_register_callback(void (*callback)(void))
{
    timer_callback = callback;
}
static int combination_check()
{
	if(combination[0] == 1 && combination[1] == 0 && combination[2] == 0)//first combination
		return 1;
	else if(combination[0] == 0 && combination[1] == 1 && combination[2] == 0)//second combination
		return 2;
	else if(combination[0] == 0 && combination[1] == 0 && combination[2] == 1)//third combination
		return 3;
	else if(combination[0] == 1 && combination[1] == 1 && combination[2] == 0)//fourth combination
		return 4;
	else if(combination[0] == 0 && combination[1] == 1 && combination[2] == 1)//five combination
		return 5;
	else if(combination[0] == 0 && combination[1] == 0 && combination[2] == 0)//error combination
		return 0;
}
int camera(void)
{
	int comb,i;
	_delay_ms(100);
	for(i=0;i<5;i++)
	{
		combination[0] = gpio_read_pin(9);
		combination[1] = gpio_read_pin(10);
		combination[2] = gpio_read_pin(11);
		
		comb = combination_check();
		_delay_ms(100);	
	}
	return comb;
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
	if(sys_time >= 80000)
		actuators_umbrella();
	sys_time++;
}
uint8_t system_jumper_check(void)
{
	gpio_debouncer();
	if(gpio_read_pin(39) == 0)
	{
		return 1;
	}
	return 0;
}
void system_reset_system_time(void)
{
	sys_time = 0;
}
void system_set_match_started(void)
{
	match_started = 1;
}
uint32_t system_get_system_time(void)
{
	return sys_time;
}
uint8_t system_get_match_started(void)
{
	return match_started;
}
uint8_t return_active_state(void)
{
	return active_state;
}
void system_init(void)
{	

	timer_register_callback(gpio_debouncer);
	_delay_ms(100);
	
	gpio_register_pin(8,GPIO_DIRECTION_INPUT,true);							//jumper
	gpio_register_pin(9,GPIO_DIRECTION_INPUT,true);							//prekidac za stranu
	gpio_register_pin(10,GPIO_DIRECTION_INPUT,true);						//camera 0 position
	gpio_register_pin(11,GPIO_DIRECTION_INPUT,true);						//camera 1 position
	gpio_register_pin(12,GPIO_DIRECTION_INPUT,true);						//camera 2 position
	//oc2a

	//testing for leds
	gpio_register_pin(0,GPIO_DIRECTION_OUTPUT,false);						//led tactic 1
	gpio_register_pin(1,GPIO_DIRECTION_OUTPUT,false);						//led tactic 2
	gpio_register_pin(2,GPIO_DIRECTION_OUTPUT,false);						//led tactic 3
	gpio_register_pin(3,GPIO_DIRECTION_OUTPUT,false);						//led tactic 4
	gpio_register_pin(4,GPIO_DIRECTION_OUTPUT,false);						//led tactic 5
	
	DDRG = 0xff;
	PORTG = 0xff;
	servo_init(50);
	timer_init(1000);
	CAN_Init(1);

	actuators_setup();
	
	while(gpio_read_pin(8))
		_delay_ms(10);
	PORTG = 0x00;
	system_reset_system_time();
	system_set_match_started();
}