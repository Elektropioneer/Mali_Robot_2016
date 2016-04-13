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
#include "odometry.h"

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
		
	return 0;
}
int camera(void)
{
	int comb,i;
	_delay_ms(100);
	for(i=0;i<5;i++)
	{
		combination[0] = gpio_read_pin(CAMERA_0_PIN);
		combination[1] = gpio_read_pin(CAMERA_1_PIN);
		combination[2] = gpio_read_pin(CAMERA_2_PIN);
		
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
void do_the_camera()
{
	static int camera_return;
	
	camera_return = camera();
	switch(camera_return)
	{
		case 1:
			active_state = ROBOT_STATE_TACTIC_ONE;
			//gpio_write_pin(0,1);
			break;
		case 2:
			active_state = ROBOT_STATE_TACTIC_TWO;
			//gpio_write_pin(1,1);
			break;
		case 3:
			active_state = ROBOT_STATE_TACTIC_THREE;
			//gpio_write_pin(2,1);
			break;
		case 4:
			active_state = ROBOT_STATE_TACTIC_FOUR;
			//gpio_write_pin(3,1);
			break;
		case 5:
			active_state = ROBOT_STATE_TACTIC_FIVE;
			//gpio_write_pin(4,1);
			break;
		case 0:
			active_state = 5;
			//gpio_write_pin(0,1);
			//gpio_write_pin(4,1);
			break;
		//maybe put default for rotate for getting the camera right
	}
}
void system_init(void)
{	

	timer_register_callback(gpio_debouncer);
	_delay_ms(100);
	
	gpio_register_pin(JUMPER_PIN,GPIO_DIRECTION_INPUT,true);							//jumper
	gpio_register_pin(SIDE_PIN,GPIO_DIRECTION_INPUT,true);								//prekidac za stranu
	gpio_register_pin(CAMERA_0_PIN,GPIO_DIRECTION_INPUT,true);							//camera 0 position
	gpio_register_pin(CAMERA_1_PIN,GPIO_DIRECTION_INPUT,true);							//camera 1 position
	gpio_register_pin(CAMERA_2_PIN,GPIO_DIRECTION_INPUT,true);							//camera 2 position

	gpio_register_pin(SENSOR_F_L_PIN,GPIO_DIRECTION_INPUT,true);						//sensor front left
	gpio_register_pin(SENSOR_F_R_PIN,GPIO_DIRECTION_INPUT,true);						//sensor front right
	gpio_register_pin(SENSOR_B_L_PIN,GPIO_DIRECTION_INPUT,true);						//sensor back left
	gpio_register_pin(SENSOR_B_R_PIN,GPIO_DIRECTION_INPUT,true);						//sensor back right
	
	
	DDRG = 0xff;
	//PORTG = 0xff;
	servo_init(50);
	timer_init(1000);
	CAN_Init(1);

	actuators_setup();
	
	while(gpio_read_pin(JUMPER_PIN))
		_delay_ms(10);
	//PORTG = 0x00;
	system_reset_system_time();
	system_set_match_started();
}
signed char checkFrontSensors(signed char sensor)
{
	if(sensor == FRONT_LEFT_SIDE)
	{
		if(gpio_read_pin(SENSOR_F_L_PIN) == TRUE)
		{
			return DETECTED;
		}
	}
	else if(sensor == FRONT_RIGHT_SIDE)
	{
		if(gpio_read_pin(SENSOR_F_R_PIN) == TRUE)
		{
			return DETECTED;
		}
	}
	else if(sensor == FRONT_ALL)
	{
		if(gpio_read_pin(SENSOR_F_L_PIN) == TRUE || gpio_read_pin(SENSOR_F_R_PIN) == TRUE)
		{
			return DETECTED;
		}
	}
	return NOT_DETECTED;
}
signed char checkRearSensors(signed char sensor)
{
	if(sensor == BACK_LEFT_SIDE)
	{
		if(gpio_read_pin(SENSOR_B_L_PIN) == TRUE)
		{
			return DETECTED;
		}
	}
	else if(sensor == BACK_RIGHT_SIDE)
	{
		if(gpio_read_pin(SENSOR_B_R_PIN) == TRUE)
		{
			return DETECTED;
		}
	}
	else if(sensor == BACK_ALL)
	{
		if(gpio_read_pin(SENSOR_B_L_PIN) == TRUE || gpio_read_pin(SENSOR_B_R_PIN) == TRUE)
		{
			return DETECTED;
		}
	}
	return NOT_DETECTED;
}