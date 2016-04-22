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
	signed char i;
	i = check_front_sensors(FRONT_ALL);
	if(i == DETECTED)
	{
		stop(HARD_STOP);
		return 1;
	}
	return 0;
}
char grabbers_down_green(uint32_t start_time)
{
	servo_set_grabbers_down();
	return 0;
}
char grabbers_up_green(uint32_t start_time)
{
	servo_set_grabbers_up();
	return 0;
}
//////////////////////////////////////////////////////////////////////////
const struct goto_fields green_tactic_one_positions[TACTIC_ONE_POSITION_COUNT] =
{
	{{280,1750},LOW_SPEED,FORWARD,green_detection_front},						//POSITION 0			MOVE FORWARD FOR THE BIG ROBOT TO GO
	{{280,1910},20,FORWARD,NULL},						//POSITION 1			MOVE BACK INFRONT OF THE BLOCKS
	{{280,1810},20,BACKWARD,NULL}

};
void greenside(void)
{
	struct odometry_position starting_position;
	uint8_t current_position = 0;
	uint8_t next_position = 0;
	uint8_t odometry_status;
	int8_t active_state = ROBOT_STATE_TACTIC_ONE;
	
	starting_position.x		= 180;
	starting_position.y		= 1320;
	starting_position.angle = 90;
	
	odometry_set_position(&starting_position);

	while(1)
	{
		switch(active_state)
		{
			case ROBOT_STATE_COLLISION:
				if(current_position == 0)
				{
					_delay_ms(200);
					while(green_tactic_one_positions[current_position].callback(0) != 0)
					_delay_ms(100);
					//next_position = current_position;
					active_state = ROBOT_STATE_TACTIC_ONE;
					break;
				}
			case ROBOT_STATE_TACTIC_ONE:
				for(current_position = next_position;current_position < TACTIC_ONE_POSITION_COUNT; current_position++)
				{
					odometry_status = odometry_move_to_position(&(green_tactic_one_positions[current_position].point), green_tactic_one_positions[current_position].speed,
					green_tactic_one_positions[current_position].direction,green_tactic_one_positions[current_position].callback);
		
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
						_delay_ms(1500);
					}
					if(current_position == 2)
					{
						while(1);
					}
				}//end for
		}//end switch
	}//end while
}//end void
