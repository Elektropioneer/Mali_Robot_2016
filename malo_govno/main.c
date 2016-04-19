#include "gpio.h"
#include "system.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "odometry.h"
#include "sides.h"
#include "actuators.h"
/*
int camera_state = 0;

static int purple_camera()
{
	//put coordinations XD
	odometry_move_straight(150,NORMAL_SPEED,NULL);
	PORTG = 0xff;
	odometry_rotate(180,LOW_SPEED,NULL);
	
	while(camera_state == 0)
	{
		camera_state = camera();		
	}
	return camera_state;
	
}
static int green_camera()
{
	//put coordinations XD
	
	while(camera_state == 0)
	{
		camera_state = camera();	
	}
	return camera_state;
}
static void led()
{
	PORTG = 0x00;
	_delay_ms(1000);
	PORTG = 0xff;
}*/
int main()
{

	system_init();
	_delay_ms(100);

	while(1)
	{
		odometry_move_straight(200,NORMAL_SPEED,NULL);
		PORTG = 0x00;
		odometry_move_straight(-200,NORMAL_SPEED,NULL);
		while(1);
	}
		/*odometry_move_straight(200,LOW_SPEED,NULL);
		PORTG = 0xff;*/
}