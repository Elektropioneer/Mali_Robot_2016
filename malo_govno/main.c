#include "gpio.h"
#include "system.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "odometry.h"
#include "sides.h"
#include "actuators.h"

int main()
{

	system_init();
	_delay_ms(100);
	
	int i,b;
	
	while(1)
	{
		/*
		if(gpio_read_pin(SIDE_PIN) == 1)
		{
			greenside();
		}
		
		else
		{	
			purpleside();
		}
		
		*/
		/*
		i = checkRearSensors(BACK_ALL);
		b = checkFrontSensors(FRONT_ALL);
		if(i == DETECTED || b == DETECTED)
		{
			PORTG = 0xff;
		}
		else
		{
			PORTG = 0x00;
		}
		*/
		
	}
}