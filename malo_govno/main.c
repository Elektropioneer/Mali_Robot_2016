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
	while(1)
	{
		servo_set_kisobran_position(-86);
		_delay_ms(4000);
		servo_set_kisobran_position(90);
		

		while(1);
		//greenside();
	}
}