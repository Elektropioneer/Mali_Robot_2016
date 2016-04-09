#include "gpio.h"
#include "system.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "odometry.h"
#include "sides.h"
#include "actuators.h"

void beginning_move(int side)
{
	int i;
	
	odometry_move_straight(160,LOW_SPEED,NULL);
	_delay_ms(100);
	odometry_move_straight(-300,LOW_SPEED,NULL);
	if(side == PURPLE_SIDE)
		odometry_rotate_for(-90,LOW_SPEED,NULL);
	else
		odometry_rotate_for(90,LOW_SPEED,NULL);
	//actuators set the left and right one
	odometry_move_straight(1015,NORMAL_SPEED,NULL);
	odometry_move_straight(-100,NORMAL_SPEED,NULL);
	odometry_rotate_for(55,NORMAL_SPEED,NULL);
	i = camera();
	
	switch(i)
	{
		case 0:
			active_state = 0;
			gpio_write_pin(0,1);
			break;	
		case 1:
			active_state = 1;
			gpio_write_pin(1,1);
			break;
		case 2:
			active_state = 2;
			gpio_write_pin(2,1);
			break;
		case 3:
			active_state = 3;
			gpio_write_pin(3,1);
			break;		
		case 4:
			active_state = 4;
			gpio_write_pin(4,1);
			break;
		case 5:	
			active_state = 5;
			gpio_write_pin(0,1);
			gpio_write_pin(4,1);
			break;
	}
}

int main()
{
	static bool done_camera = false;

	system_init();

	while(1)
	{
		//put if for the side checker
		if(!done_camera)
			beginning_move(GREEN_SIDE);
			done_camera = true;
		PORTG = 0xff;
		greenside();
	}
}