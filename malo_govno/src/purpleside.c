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
	signed char i;
	i = check_front_sensors(FRONT_ALL);
	if(i == DETECTED)
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
	{{280,280},LOW_SPEED,FORWARD,purple_detection_front},						//POSITION 0			MOVE FORWARD FOR THE BIG ROBOT TO GO
	{{280,140},20,FORWARD,NULL},						//POSITION 1			MOVE BACK INFRONT OF THE BLOCKS
	{{280,190},10,BACKWARD,NULL}

};
void purpleside(void)
{
	struct odometry_position starting_position;
	uint8_t current_position = 0;
	uint8_t next_position = 0;
	uint8_t odometry_status;
	int8_t active_state = ROBOT_STATE_TACTIC_ONE;
	
	starting_position.x		= 180;
	starting_position.y		= 680;
	starting_position.angle = 90;
	
	odometry_set_position(&starting_position);
	
	odometry_move_straight(-200,LOW_SPEED,NULL);
	_delay_ms(1000);
	odometry_rotate(-180,LOW_SPEED,NULL);
	_delay_ms(1000);
	while(1)
	{
		switch(active_state)
		{
			case ROBOT_STATE_COLLISION:
				if(current_position == 0)
				{
					_delay_ms(200);
					while(purple_tactic_one_positions[current_position].callback(0) != 0)
					_delay_ms(100);
					//next_position = current_position;
					active_state = ROBOT_STATE_TACTIC_ONE;
					break;
				}
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
						_delay_ms(3000);
					}
					else if(current_position == 1)
					{
						_delay_ms(2000);
					}
					if(current_position == 2)
					{
						while(1);
					}
				}//end for
		}//end switch
	}//end while
}//end void
