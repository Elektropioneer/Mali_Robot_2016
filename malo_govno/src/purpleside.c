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

const struct goto_fields purple_tactic_one_positions[TACTIC_ONE_POSITION_COUNT] = 
{
	{{85,1220},NORMAL_SPEED,FORWARD,NULL}	
};
const struct goto_fields purple_tactic_two_positions[TACTIC_TWO_POSITION_COUNT] = 
{
	{{85,1220},NORMAL_SPEED,FORWARD,NULL}	
};
const struct goto_fields purple_tactic_three_positions[TACTIC_THREE_POSITION_COUNT] = 
{
	{{85,1220},NORMAL_SPEED,FORWARD,NULL}	
};
const struct goto_fields purple_tactic_four_positions[TACTIC_FOUR_POSITION_COUNT] = 
{
	{{85,1220},NORMAL_SPEED,FORWARD,NULL}	
};
const struct goto_fields purple_tactic_five_positions[TACTIC_FIVE_POSITION_COUNT] =
{
	{{85,1220},NORMAL_SPEED,FORWARD,NULL}	
};
void purpleside(void)
{
	struct odometry_position starting_position;
	uint8_t current_position = 0;
	uint8_t next_position	 = 0;
	uint8_t odometry_status;
	uint8_t active_state = return_active_state();
	
	starting_position.x		= 85;
	starting_position.y		= 1020;
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
					if(current_position == 0)
					{
						while(1);
					}
				}//end for
			case ROBOT_STATE_TACTIC_TWO:	
				for(current_position = next_position;current_position < TACTIC_TWO_POSITION_COUNT; current_position++)
				{
					odometry_status = odometry_move_to_position(&(purple_tactic_two_positions[current_position].point), purple_tactic_two_positions[current_position].speed,
					purple_tactic_two_positions[current_position].direction,purple_tactic_two_positions[current_position].callback);
					if(odometry_status == ODOMETRY_FAIL)
					{
						break;
					}
					if(current_position == 0)
					{
						while(1);
					}
				}//end for
			case ROBOT_STATE_TACTIC_THREE:
				for(current_position = next_position;current_position < TACTIC_THREE_POSITION_COUNT; current_position++)
				{
					odometry_status = odometry_move_to_position(&(purple_tactic_three_positions[current_position].point), purple_tactic_three_positions[current_position].speed,
					purple_tactic_three_positions[current_position].direction,purple_tactic_three_positions[current_position].callback);
					if(odometry_status == ODOMETRY_FAIL)
					{
						break;
					}
					if(current_position == 0)
					{
						while(1);
					}
				}//end for
			case ROBOT_STATE_TACTIC_FOUR:
				for(current_position = next_position;current_position < TACTIC_FOUR_POSITION_COUNT; current_position++)
				{
					odometry_status = odometry_move_to_position(&(purple_tactic_four_positions[current_position].point), purple_tactic_four_positions[current_position].speed,
					purple_tactic_four_positions[current_position].direction,purple_tactic_four_positions[current_position].callback);
					if(odometry_status == ODOMETRY_FAIL)
					{
						break;
					}
					if(current_position == 0)
					{
						while(1);
					}
				}//end for
			case ROBOT_STATE_TACTIC_FIVE:
				for(current_position = next_position;current_position < TACTIC_FIVE_POSITION_COUNT; current_position++)
				{
					odometry_status = odometry_move_to_position(&(purple_tactic_five_positions[current_position].point), purple_tactic_five_positions[current_position].speed,
					purple_tactic_five_positions[current_position].direction,purple_tactic_five_positions[current_position].callback);
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
	}//end while
}//end void
