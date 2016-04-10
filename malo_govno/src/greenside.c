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

const struct goto_fields green_camera_move[TACTIC_CAMERA_POSITION_COUNT] =
{
	//zoviti ducija i namestiti
	//ili samo izracunati 
	
	
	{{85,1180},LOW_SPEED,FORWARD,NULL},         // gura prvi pak
	{{85,880},NORMAL_SPEED,BACKWARD,NULL},		//vraca se ispred kocki
	{{1100,980},NORMAL_SPEED,FORWARD,NULL},		//gura kocke
	{{900,980},NORMAL_SPEED,BACKWARD,NULL}
};
const struct goto_fields green_tactic_one_positions[TACTIC_ONE_POSITION_COUNT] =
{
	{{85,1220},NORMAL_SPEED,FORWARD,NULL}
};
const struct goto_fields green_tactic_two_positions[TACTIC_TWO_POSITION_COUNT] =
{
	{{85,1220},NORMAL_SPEED,FORWARD,NULL}
};
const struct goto_fields green_tactic_three_positions[TACTIC_THREE_POSITION_COUNT] =
{
	{{85,1220},NORMAL_SPEED,FORWARD,NULL}
};
const struct goto_fields green_tactic_four_positions[TACTIC_FOUR_POSITION_COUNT] =
{
	{{85,1220},NORMAL_SPEED,FORWARD,NULL}
};
const struct goto_fields green_tactic_five_positions[TACTIC_FIVE_POSITION_COUNT] =
{
	{{85,1220},NORMAL_SPEED,FORWARD,NULL}
};

void greenside(void)
{
	struct odometry_position starting_position;
	
	uint8_t current_position = 0; 
	uint8_t next_position = 0; 
	uint8_t odometry_status;
	uint8_t active_state;
	//uint8_t active_state = ROBOT_STATE_TACTIC_ONE;

	starting_position.x     = 85;
	starting_position.y     = 670;
	starting_position.angle = -90;
	 
	odometry_set_position(&starting_position);
	 
	for(current_position = next_position;current_position < TACTIC_CAMERA_POSITION_COUNT; current_position++)
	{
		odometry_status = odometry_move_to_position(&(green_camera_move[current_position].point), green_camera_move[current_position].speed,
		green_camera_move[current_position].direction,green_camera_move[current_position].callback);
		if(odometry_status == ODOMETRY_FAIL)
		{
			break;
		}
		if(current_position == 3)
		{
			odometry_rotate_for(85,NORMAL_SPEED,NULL);
			_delay_ms(200);
			//do the camera work
			do_the_camera();
			_delay_ms(500);
		}
	}//end for
	 
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
			case ROBOT_STATE_TACTIC_TWO:
				for(current_position = next_position;current_position < TACTIC_TWO_POSITION_COUNT; current_position++)
				{
					odometry_status = odometry_move_to_position(&(green_tactic_two_positions[current_position].point), green_tactic_two_positions[current_position].speed,
					green_tactic_two_positions[current_position].direction,green_tactic_two_positions[current_position].callback);
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
					odometry_status = odometry_move_to_position(&(green_tactic_three_positions[current_position].point), green_tactic_three_positions[current_position].speed,
					green_tactic_three_positions[current_position].direction,green_tactic_three_positions[current_position].callback);
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
					odometry_status = odometry_move_to_position(&(green_tactic_four_positions[current_position].point), green_tactic_four_positions[current_position].speed,
					green_tactic_four_positions[current_position].direction,green_tactic_four_positions[current_position].callback);
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
					odometry_status = odometry_move_to_position(&(green_tactic_five_positions[current_position].point), green_tactic_five_positions[current_position].speed,
					green_tactic_five_positions[current_position].direction,green_tactic_five_positions[current_position].callback);
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