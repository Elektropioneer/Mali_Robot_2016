#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "system.h"
#include "odometry.h"
#include "can.h"
#include "sides.h"
#include "uart.h"
#include "actuators.h"
#include "gpio.h"

//////////////////////////////////////////////////////////////////////////
//																		//
//					   DETEKCIJA/CALLBACK FUNKCIJE						//
//																		//
//////////////////////////////////////////////////////////////////////////

char purple_detection_front(uint32_t start_time)
{
	if(checkFrontSensors(FRONT_ALL) == DETECTED)
	{
		stop(HARD_STOP);
		return 1;
	}
	return 0;
}
char purple_detection_front_left(uint32_t start_time)
{
	if(checkFrontSensors(FRONT_LEFT_SIDE) == DETECTED)
	{
		stop(HARD_STOP);
		return 1;
	}
	return 0;
}
char purple_detection_front_right(uint32_t start_time)
{
	if(checkFrontSensors(FRONT_RIGHT_SIDE) == DETECTED)
	{
		stop(HARD_STOP);
		return 1;
	}
	return 0;
}
char purple_detection_back(uint32_t start_time)
{
	if(checkRearSensors(BACK_ALL) == DETECTED)
	{
		stop(HARD_STOP);
		return 1;
	}
	return 0;
}
char purple_detection_back_left(uint32_t start_time)
{
	if(checkRearSensors(BACK_LEFT_SIDE) == DETECTED)
	{
		stop(HARD_STOP);
		return 1;
	}
	return 0;
}
char purple_detection_back_right(uint32_t start_time)
{
	if(checkRearSensors(BACK_RIGHT_SIDE) == DETECTED)
	{
		stop(HARD_STOP);
		return 1;
	}
	return 0;
}


char grabbers_down_purple(uint32_t start_time)
{
	servo_set_grabbers_down();
	return 0;
}
char grabbers_up_purple(uint32_t start_time)
{
	servo_set_grabbers_up();
	return 0;
}
//////////////////////////////////////////////////////////////////////////
const struct goto_fields purple_tactic_one_positions[TACTIC_ONE_POSITION_COUNT] = 
{
	{{185,1180},LOW_SPEED,FORWARD,NULL},						//POSITION 0			MOVE FORWARD FOR THE BIG ROBOT TO GO
	{{185,880},LOW_SPEED,BACKWARD,NULL},						//POSITION 1			MOVE BACK INFRONT OF THE BLOCKS 
	{{1100,980},LOW_SPEED,FORWARD,grabbers_down_purple},		//POSITION 2			PUSH THE BLOCKS TO THE GATE
	{{1000,980},LOW_SPEED,BACKWARD,NULL},						//POSITION 3			GET A LITTLE BACK
	{{280,1520},LOW_SPEED,FORWARD,NULL},						//POSITION 4			GRAB 2 PACKS 
	{{220,1420},20,FORWARD,NULL},								//POSITION 5			ROTATING WITH 2 PACKS 
	{{140,1100},20,FORWARD,NULL},								//POSITION 6			MOVE FORWARD PACKS
	{{280,200},90,BACKWARD,grabbers_up_purple},					//POSITION 7			GOING INFRONT DOORS
	{{280,90},20,FORWARD,NULL}									//POSITION 8			GOING INTO THE DOORS

};
void purpleside(void)
{
	struct odometry_position starting_position;
	uint8_t current_position = 0;
	uint8_t next_position = 0;
	uint8_t odometry_status;
	int8_t active_state = ROBOT_STATE_TACTIC_ONE;
	
	starting_position.x		= 180;
	starting_position.y		= 1010;
	starting_position.angle = 90;
	
	odometry_set_position(&starting_position);
	
	while(1)
	{
		switch(active_state)
		{
			case ROBOT_STATE_TACTIC_ONE:
				for(current_position = next_position;current_position < TACTIC_ONE_POSITION_COUNT; current_position++)
				{
					odometry_status = odometry_move_to_position(&(purple_tactic_one_positions[current_position].point), purple_tactic_one_positions[current_position].speed,
																purple_tactic_one_positions[current_position].direction,purple_tactic_one_positions[current_position].callback); 

					if(odometry_status == ODOMETRY_FAIL)
					{
						break;
					}
					else if(current_position == 1 || current_position == 3)
					{
						_delay_ms(1000);
					}
					else if(current_position == 6)
					{
						servo_set_right_grabber_position(100);
						_delay_ms(800);
						odometry_rotate(45,LOW_SPEED,NULL);
						_delay_ms(500);
						servo_set_left_grabber_position(20);
						_delay_ms(800);
						odometry_move_straight(-200,LOW_SPEED,NULL);
						_delay_ms(500);
						odometry_set_angle(90,LOW_SPEED,NULL);
						
					}
					else if(current_position == 10 || current_position == 11)
					{
						_delay_ms(1000);
					}
					else if(current_position == 8)
					{
						odometry_move_straight(-200,LOW_SPEED,NULL);
						_delay_ms(800);
						odometry_rotate(-90,LOW_SPEED,NULL);
						_delay_ms(500);
						odometry_move_straight(300,LOW_SPEED,NULL);
						_delay_ms(800);
						odometry_rotate(90,LOW_SPEED,NULL);
						_delay_ms(500);
						odometry_move_straight(190,20,NULL);
						_delay_ms(1000);
						odometry_move_straight(-150,20,NULL);
						while(1);
					}
				}//end for
		}//end switch
	}//end while
}//end void
