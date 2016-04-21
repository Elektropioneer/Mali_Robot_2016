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

//////////////////////////////////////////////////////////////////////////
const struct goto_fields purple_tactic_one_positions[TACTIC_ONE_POSITION_COUNT] = 
{
	{{185,1180},LOW_SPEED,FORWARD,NULL},                              //POSITION 0			MOVE FORWARD FOR THE BIG ROBOT TO GO
	{{185,880},LOW_SPEED,BACKWARD,NULL},		                     //POSITION 1			MOVE BACK INFRONT OF THE BLOCKS 
	{{1100,980},LOW_SPEED,FORWARD,purple_detection_front},		    //POSITION 2			PUSH THE BLOCKS TO THE GATE
	{{185,980},LOW_SPEED,BACKWARD,purple_detection_back}		    //POSITION 3			GET  BACK
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
					else if(current_position == 3)
					{
						while(1);
					}
				}//end for
		}//end switch
	}//end while
}//end void
