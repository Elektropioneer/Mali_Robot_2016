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

	//test servo_left, right actuator
	//test read pin 15
	//test camera
	
	while(1)
	{
		if(gpio_read_pin(15) == 1)
		{
			greenside();
		}
		else
		{	
			purpleside();
		}
	}
}