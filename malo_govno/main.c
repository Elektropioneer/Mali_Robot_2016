#include "gpio.h"
#include "system.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "odometry.h"


int main()
{
	int i = 0;	
	
	system_init();
	
	while(1)
	{
		//middle button
		while(i == 0)
		{
			gpio_register_pin(31,GPIO_DIRECTION_INPUT,true);
			gpio_debouncer();
			if(gpio_read_pin(31) == 1)
			{
				i = 1;
				//_delay_ms(1000);
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
				greenside();
			}
		}
	}
}