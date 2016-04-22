#include "gpio.h"
#include "system.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "odometry.h"
#include "sides.h"
#include "actuators.h"

int set = 1;

int main()
{

	system_init();
	_delay_ms(100);
	/*	
	while(set)
	{
		if(sides_switch_check() == 0)
		{
			set = 2;
			greenside();	
		}
		else
		{
			set = 3;
			purpleside();	
		}
	}
	while(1)
	{
		if(set == 2)
		{
			greenside();
		}
		else if(set == 3)
		{
			purpleside();
		}
	}*/
	
	signed char i;
	while(1)
	{
		i=check_back_sensors(BACK_ALL);
		if(i == DETECTED)
		{
			PORTG = 0xff;
		}
		else
		{
			PORTG = 0x00;
		}
	}
}