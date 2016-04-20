#include "gpio.h"
#include "system.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "odometry.h"
#include "sides.h"
#include "actuators.h"

int camera_state = 0;

static int purple_camera()
{
	//put coordinations XD
	odometry_move_straight(150,50,NULL);					//ide napred zbog velikog
	_delay_ms(2000);
	PORTG = 0xff;
	odometry_move_straight(-220,LOW_SPEED,NULL);			//vraca se nazad
	_delay_ms(500);
	odometry_rotate(90,LOW_SPEED,NULL);						//rotira se za kocke
	_delay_ms(1000);
	odometry_move_straight(1000,100,NULL);					//gura kocke
	odometry_move_straight(-100,LOW_SPEED,NULL);             //vraca se unazad da se rotira za poziciju kamere
	_delay_ms(500);
	odometry_rotate(80,LOW_SPEED,NULL);                     //rotira se da dodje u poziciju za slikanje kamere
	/*
	while(camera_state == 0)
	{
		camera_state = camera();		
	}
	return camera_state;
	*/
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
}
int main()
{
	int camera = 1;


	system_init();
	_delay_ms(100);

	while(1)
	{
		/*if(camera)
		{
			purple_camera();	
			camera = 0;
			_delay_ms(3000);
		}*/
		purpleside();
	}
}