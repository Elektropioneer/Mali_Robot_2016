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
	//90
	
	while(1)
	{
		PORTG = 0xff;
		_delay_ms(1000);
		PORTG = 0x00;
		_delay_ms(1000);
		//greenside();
	}
}