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
const struct goto_fields green_camera[TACTIC_CAMERA_POSITION] =
{
	{{85,1180},LOW_SPEED,FORWARD,NULL},         	// gura prvi pak
	{{85,880},LOW_SPEED,BACKWARD,NULL},		//vraca se ispred kocki
	{{1100,980},LOW_SPEED,FORWARD,NULL},		//gura kocke
	{{900,980},LOW_SPEED,BACKWARD,NULL}		//vraca se nazad
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
	{{1380,1850},NORMAL_SPEED,FORWARD,NULL},	//POSITION 0			ide iza pakova
	{{200,1350},NORMAL_SPEED,FORWARD,NULL},		//POSITION 1			gura pakove
	{{600,1500},NORMAL_SPEED,BACKWARD,NULL},	//POSITION 2			vraca se nazad
	{{450,1840},NORMAL_SPEED,FORWARD,NULL},		//POSITION 3			priprema za guranje
	{{40,1520},NORMAL_SPEED,FORWARD,NULL},		//POSITION 4			do pakova
	{{80,1100},LOW_SPEED,FORWARD,NULL},			//POSITION 5			gura ih do kraja
	{{80,1500},LOW_SPEED,BACKWARD,NULL},		//POSITION 6			vraca se malo nazad
	{{900,1450},NORMAL_SPEED,FORWARD,NULL},		//POSITION 7			priprema za pak
	{{180,950},NORMAL_SPEED,FORWARD,NULL},		//POSITION 8			gura pak
	{{280,1300},NORMAL_SPEED,BACKWARD,NULL},	//POSITION 9			vraca se nazad
	{{0,1300},30,FORWARD,NULL},					//POSITION 10			skolni se
	{{280,1300},NORMAL_SPEED,BACKWARD,NULL},	//POSITION 11			vraca se nazad
	{{280,200},NORMAL_SPEED,FORWARD,NULL},		//POSITION 12			vrata 1 ispred
	{{280,60},10, FORWARD,NULL},				//POSITION 13			gura vrata
	{{280,250},LOW_SPEED,BACKWARD,NULL},		//POSITION 14			vraca se nazad
	{{600,250},50,FORWARD,NULL},				//POSITION 15			ide ispred druge
	{{600,200},50,FORWARD,NULL},				//POSITION 16			vrata 2 ispred
	{{600,60},10, FORWARD,NULL},				//POSITION 17			vrata 2
	{{600,100},LOW_SPEED,BACKWARD,NULL}			//POSITION 19			cao
};
const struct goto_fields green_tactic_four_positions[TACTIC_FOUR_POSITION_COUNT] =
{
	{{1380,1850},NORMAL_SPEED,FORWARD,NULL},	//POSITION 0			ide iza pakova
	{{200,1350},NORMAL_SPEED,FORWARD,NULL},		//POSITION 1			gura pakove
	{{600,1500},NORMAL_SPEED,BACKWARD,NULL},	//POSITION 2			vraca se nazad
	{{450,1840},NORMAL_SPEED,FORWARD,NULL},		//POSITION 3			priprema za guranje
	{{40,1520},NORMAL_SPEED,FORWARD,NULL},		//POSITION 4			do pakova
	{{80,1100},LOW_SPEED,FORWARD,NULL},			//POSITION 5			gura ih do kraja
	{{80,1500},LOW_SPEED,BACKWARD,NULL},		//POSITION 6			vraca se malo nazad
	{{900,1450},NORMAL_SPEED,FORWARD,NULL},		//POSITION 7			priprema za pak
	{{180,950},NORMAL_SPEED,FORWARD,NULL},		//POSITION 8			gura pak
	{{280,1300},NORMAL_SPEED,BACKWARD,NULL},	//POSITION 9			vraca se nazad
	{{0,1300},30,FORWARD,NULL},					//POSITION 10			skolni se
	{{280,1300},NORMAL_SPEED,BACKWARD,NULL},	//POSITION 11			vraca se nazad
	{{280,200},NORMAL_SPEED,FORWARD,NULL},		//POSITION 12			vrata 1 ispred
	{{280,60},10, FORWARD,NULL},				//POSITION 13			gura vrata
	{{280,250},LOW_SPEED,BACKWARD,NULL},		//POSITION 14			vraca se nazad
	{{600,250},50,FORWARD,NULL},				//POSITION 15			ide ispred druge
	{{600,200},50,FORWARD,NULL},				//POSITION 16			vrata 2 ispred
	{{600,60},10, FORWARD,NULL},				//POSITION 17			vrata 2
	{{600,100},LOW_SPEED,BACKWARD,NULL}			//POSITION 19			cao
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
					if(current_position == 10)
					{
						_delay_ms(10000);
					}
					else if(current_position == 11 || current_position ==  15)
					{
						_delay_ms(2000);
					}
					else if(current_position == 18)
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
					if(current_position == 10)
					{
						_delay_ms(10000);
					}
					else if(current_position == 11 || current_position ==  15)
					{
						_delay_ms(2000);
					}
					else if(current_position == 18)
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