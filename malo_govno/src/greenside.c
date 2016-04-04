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

const struct goto_fields green_tactic_one_positions[TACTIC_ONE_POSITION_COUNT] = 
{
	{{85,500}, 110, FORWARD, NULL}
};

void greenside(void)
{
	struct odometry_position starting_position;
	
	uint8_t current_position = 0; 
	uint8_t next_position = 0; 
	uint8_t odometry_status;
	uint8_t active_state = ROBOT_STATE_TACTIC_ONE;

	starting_position.x     = 85;
	starting_position.y     = 670;
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
					if(current_position == 0)
					{
						while(1);
					}
				}//end for
		}//end switch
	}//end of while
}//end of greenside