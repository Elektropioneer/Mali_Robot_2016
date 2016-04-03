#include "gpio.h"
#include "system.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "odometry.h"
#include "actuators.h"

int main()
{
	system_init();
	PORTG = 0xff;
	while(1)
	{
		_delay_ms(1500);
		servo_set_position(-40);
		_delay_ms(1500);
		servo_set_position(40);
		_delay_ms(1500);
		
		/*_delay_ms(1000);
		//middle button
		while(i == 0)
		{
			gpio_register_pin(31,GPIO_DIRECTION_INPUT,true);
			gpio_debouncer();
			if(gpio_read_pin(31) == 1)
			{
				i = 1;
				_delay_ms(1000);
			}
			else
			{
				i = 0;
			}
		}	
		if(i == 1)
		{
			while(1)
			{
				
				PORTG = 0xff;
				//greenside();
				
				odometry_move_straight(300,NORMAL_SPEED,NULL);
				odometry_rotate_for(90,LOW_SPEED,NULL);
				odometry_move_straight(100,NORMAL_SPEED,NULL);
				odometry_rotate_for(-90,LOW_SPEED,NULL);
				odometry_move_straight(500,NORMAL_SPEED,NULL);
				
				while(1);
				
				servo_set_position(20);
				_delay_ms(2000);
			}
		}*/
	}
}