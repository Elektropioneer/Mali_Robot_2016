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
	_delay_ms(1000);

	//test servo_left, right actuator
	//test read pin 15
	//test camera
	
	while(1)
	{
		/*if(gpio_read_pin(14) == 1)
		{
			PORTG = 0xff;
		}
		
		else
		{	
			PORTG = 0x00;
		}*/
		//100 up
		//250 down
		
		PORTG = 0xff;
		servo_set_grabbers_down();
		while(1);
	}
}