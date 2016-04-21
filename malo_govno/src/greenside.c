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

char green_detection_front(uint32_t start_time)
{
	if(checkFrontSensors(FRONT_ALL) == DETECTED)
	{
		stop(HARD_STOP);
		return 1;
	}
	return 0;
}
char green_detection_front_left(uint32_t start_time)
{
	if(checkFrontSensors(FRONT_LEFT_SIDE) == DETECTED)
	{
		stop(HARD_STOP);
		return 1;
	}
	return 0;
}
char green_detection_front_right(uint32_t start_time)
{
	if(checkFrontSensors(FRONT_RIGHT_SIDE) == DETECTED)
	{
		stop(HARD_STOP);
		return 1;
	}
	return 0;
}
char green_detection_back(uint32_t start_time)
{
	if(checkRearSensors(BACK_ALL) == DETECTED)
	{
		stop(HARD_STOP);
		return 1;
	}
	return 0;
}
char green_detection_back_left(uint32_t start_time)
{
	if(checkRearSensors(BACK_LEFT_SIDE) == DETECTED)
	{
		stop(HARD_STOP);
		return 1;
	}
	return 0;
}
char green_detection_back_right(uint32_t start_time)
{
	if(checkRearSensors(BACK_RIGHT_SIDE) == DETECTED)
	{
		stop(HARD_STOP);
		return 1;
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////
const struct goto_fields green_tactic_one_positions[TACTIC_ONE_POSITION_COUNT] =
{
	{{185,820},LOW_SPEED,FORWARD,NULL},         	             // gura prvi pak
	{{185,1120},LOW_SPEED,BACKWARD,NULL},		                //vraca se ispred kocki
	{{1100,1020},LOW_SPEED,FORWARD,green_detection_front},		//gura kocke
	{{900,1020},LOW_SPEED,BACKWARD,green_detection_back}		//vraca se nazad
};
void greenside(void)
{
	struct odometry_position starting_position;
	
	uint8_t current_position = 0; 
	uint8_t next_position = 0; 
	uint8_t odometry_status;
	uint8_t active_state = ROBOT_STATE_TACTIC_ONE;

	starting_position.x     = 185;
	starting_position.y     = 990;
	starting_position.angle = -90;
	 
	odometry_set_position(&starting_position);
	 
	while(1)
	{
		switch(active_state)
		{
			case ROBOT_STATE_TACTIC_ONE:
				for(current_position = next_position; current_position < TACTIC_ONE_POSITION_COUNT; current_position++)
				{
					odometry_status = odometry_move_to_position(&green_tactic_one_positions[current_position].point, green_tactic_one_positions[current_position].speed,
																green_tactic_one_positions[current_position].direction, green_tactic_one_positions[current_position].callback);
					if(odometry_status == ODOMETRY_FAIL)
					{
						break;
					}
					if(current_position == 3)
					{
						while(1);
					}
				}//end for
		}//end switch
	}//end of while
}//end of greenside