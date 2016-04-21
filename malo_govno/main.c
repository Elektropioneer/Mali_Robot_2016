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

	while(1)
	{
		if(gpio_read_pin(SIDE_PIN))
		{
			purpleside();
		}
		else
		{
			greenside();
		}
	}
}