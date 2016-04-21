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
const struct goto_fields purple_camera[TACTIC_CAMERA_POSITION] = 
{
	{{185,1180},LOW_SPEED,FORWARD,NULL},        //POSITION 0			MOVE FORWARD FOR THE BIG ROBOT TO GO
	{{185,880},LOW_SPEED,BACKWARD,NULL},		//POSITION 1			MOVE BACK INFRONT OF THE BLOCKS 
	{{1100,980},LOW_SPEED,FORWARD,NULL},		//POSITION 2			PUSH THE BLOCKS TO THE GATE
	{{1000,980},LOW_SPEED,BACKWARD,NULL}		//POSITION 3			GET A LITTLE BACK, AND EXECUTE CAMERA FUNCTION
};
const struct goto_fields purple_tactic_one_positions[TACTIC_ONE_POSITION_COUNT] =
{
	{{900,1450},NORMAL_SPEED,FORWARD,NULL},	//POSITION 0			GO TO THE PURPLE STAR  WITH SERVO
	{{200,1040},NORMAL_SPEED,FORWARD,NULL},		//POSITION 1			PUSH THEM TO START
	{{500,1250},NORMAL_SPEED,BACKWARD,NULL},	//POSITION 2			GO BACK BEHIND THE 2 STARS		
	{{620,1780},NORMAL_SPEED,BACKWARD,NULL},	//POSITION 3			GO TO THE RIGHT AND ALIGN FOR THE STARS TO PUSH TO START
	{{200,1560},LOW_SPEED,FORWARD,NULL},		//POSITION 4			GO ALIGN IN A STRAIGHT LINE FOR THE STARS
	{{125,1180},30,FORWARD,NULL},				//POSITION 5			PUSHING THE STARS TO THE STARTING AREA WE HOPE IT WONT FUCK UP
	{{125,1300},30,FORWARD,NULL},				//POSITION 6			GOING BACK FOR ALIGN FOR THE DOORS
	{{650,1250},30,FORWARD,NULL},				//POSITION 7			GOING INFRONT OF THE DOORS BUT NEAR TH ESTARTING AREA
	{{650,200},30,FORWARD,NULL},			//POSITION 8			GOING INFRONT OF DOOR NUMBER 2
	{{650,80},30,FORWARD,NULL}					//POSITION 9			PUSHING THE DOOR NUMBER 2 !!MAKE SURE THIS IS K!!
		
	//jos dodati
};
const struct goto_fields purple_tactic_two_positions[TACTIC_TWO_POSITION_COUNT] = 
{
	{{1150,1600},NORMAL_SPEED,FORWARD,NULL},	//POSITION 0			GO TO THE PURPLE STAR  WITH SERVO
	{{200,1040},NORMAL_SPEED,FORWARD,NULL},		//POSITION 1			PUSH THEM TO START
	{{500,1250},NORMAL_SPEED,BACKWARD,NULL},	//POSITION 2			GO BACK BEHIND THE 2 STARS
	{{620,1780},NORMAL_SPEED,BACKWARD,NULL},	//POSITION 3			GO TO THE RIGHT AND ALIGN FOR THE STARS TO PUSH TO START
	{{200,1560},LOW_SPEED,FORWARD,NULL},		//POSITION 4			GO ALIGN IN A STRAIGHT LINE FOR THE STARS
	{{125,1180},30,FORWARD,NULL},				//POSITION 5			PUSHING THE STARS TO THE STARTING AREA WE HOPE IT WONT FUCK UP
	{{125,1300},30,FORWARD,NULL},				//POSITION 6			GOING BACK FOR ALIGN FOR THE DOORS
	{{650,1250},30,FORWARD,NULL},				//POSITION 7			GOING INFRONT OF THE DOORS BUT NEAR TH ESTARTING AREA
	{{650,200},30,FORWARD,NULL},				//POSITION 8			GOING INFRONT OF DOOR NUMBER 2
	{{650,80},30,FORWARD,NULL}					//POSITION 9			PUSHING THE DOOR NUMBER 2 !!MAKE SURE THIS IS K!!
	
	//jos dodati
};
const struct goto_fields purple_tactic_three_positions[TACTIC_THREE_POSITION_COUNT] = 
{
	{{1490,1730},NORMAL_SPEED,FORWARD,NULL},	//POSITION 0			MOVE BEHIND THE STAR IN THE MIDDLE
	{{300,1400},NORMAL_SPEED,FORWARD,NULL},		//POSITION 1			MOVE STARS NEXT TO THE START POSITION WITH OTHER STARS
	{{620,1780},NORMAL_SPEED,BACKWARD,NULL},	//POSITION 2			GO TO THE RIGHT AND ALIGN FOR THE STARS TO PUSH TO START
	{{165,1560},LOW_SPEED,FORWARD,NULL},		//POSITION 3			GO ALIGN IN A STRAIGHT LINE FOR THE STARS
	{{125,1180},30,FORWARD,NULL},				//POSITION 4			PUSHING THE STARS TO THE STARTING AREA WE HOPE IT WONT FUCK UP
	{{165,1280},NORMAL_SPEED,BACKWARD,NULL},	//POSITION 5			GOING A BIT BACK NOT TO GET STUCK
	{{1020,1450},NORMAL_SPEED,FORWARD,NULL},	//POSITION 6			GOING TO THE POSITION TO PUSH THE LAST STAR TO START
	{{360,1100},NORMAL_SPEED,FORWARD,NULL},		//POSITION 7			PUSH THE LAST STAR TO START
	{{540,1360},30,BACKWARD,NULL},				//POSITION 8			GOING BACK TO ALIGN FOR THE WAITING ON THE BIG ROBOT
	{{100,1360},LOW_SPEED,FORWARD,NULL},		//POSITION 9			GO TO THE WALL SO THE BIG ROBOT CAN GO PICK UP THE FISHES
	{{650,1360},NORMAL_SPEED,BACKWARD,NULL},	//POSITION 10			GOING TO ALIGN FOR THE DOORS
	{{650,200},NORMAL_SPEED,FORWARD,NULL},		//POSITION 11			GOING INFRONT OF DOOR NUMBER 2
	{{650,80},30,FORWARD,NULL},					//POSITION 12			PUSHING THE DOOR NUMBER 2 !!MAKE SURE THIS IS K!!
	{{650,300},LOW_SPEED,BACKWARD,NULL}, 		//POSITION 13			GOING BACK FROM THE DOOR NUMBER 2
	{{350,300},LOW_SPEED,FORWARD,NULL},		    //POSITION 14			GOING INFRONT OF DOOR NUMBER 1
	{{350,200},NORMAL_SPEED,FORWARD,NULL},		//POSITION 15			GOING INFRONT OF DOOR NUMBER 1 BUT CLOSE
	{{350,80},30,FORWARD,NULL},					//POSITION 16			PUSH DOWN DOOR NUMBER 1
	{{350,400},LOW_SPEED,BACKWARD,NULL},		//POSITION 17			GO BACK AND WAIT FOR OPENING THE UMBRELLA IF THE SERVO WORKS
	{{100,400},LOW_SPEED,BACKWARD,NULL}			//POSITION 18			GO TO SLEEP TO THE LEFT OF THE STARTING AREA		
};
const struct goto_fields purple_tactic_four_positions[TACTIC_FOUR_POSITION_COUNT] = 
{
	{{1490,1730},NORMAL_SPEED,FORWARD,NULL},	//POSITION 0			MOVE BEHIND THE STAR IN THE MIDDLE
	{{300,1400},NORMAL_SPEED,FORWARD,NULL},		//POSITION 1			MOVE STARS NEXT TO THE START POSITION WITH OTHER STARS
	{{620,1780},NORMAL_SPEED,BACKWARD,NULL},	//POSITION 2			GO TO THE RIGHT AND ALIGN FOR THE STARS TO PUSH TO START
	{{165,1560},LOW_SPEED,FORWARD,NULL},		//POSITION 3			GO ALIGN IN A STRAIGHT LINE FOR THE STARS
	{{125,1180},30,FORWARD,NULL},				//POSITION 4			PUSHING THE STARS TO THE STARTING AREA WE HOPE IT WONT FUCK UP
	{{165,1280},NORMAL_SPEED,BACKWARD,NULL},	//POSITION 5			GOING A BIT BACK NOT TO GET STUCK
	{{1020,1450},NORMAL_SPEED,FORWARD,NULL},	//POSITION 6			GOING TO THE POSITION TO PUSH THE LAST STAR TO START
	{{360,1100},NORMAL_SPEED,FORWARD,NULL},		//POSITION 7			PUSH THE LAST STAR TO START
	{{540,1360},30,BACKWARD,NULL},				//POSITION 8			GOING BACK TO ALIGN FOR THE WAITING ON THE BIG ROBOT
	{{100,1360},LOW_SPEED,FORWARD,NULL},		//POSITION 9			GO TO THE WALL SO THE BIG ROBOT CAN GO PICK UP THE FISHES
	{{650,1360},NORMAL_SPEED,BACKWARD,NULL},	//POSITION 10			GOING TO ALIGN FOR THE DOORS
	{{650,200},NORMAL_SPEED,FORWARD,NULL},		//POSITION 11			GOING INFRONT OF DOOR NUMBER 2
	{{650,80},30,FORWARD,NULL},					//POSITION 12			PUSHING THE DOOR NUMBER 2 !!MAKE SURE THIS IS K!!
	{{650,300},LOW_SPEED,BACKWARD,NULL}, 		//POSITION 13			GOING BACK FROM THE DOOR NUMBER 2
	{{350,300},LOW_SPEED,FORWARD,NULL},		    //POSITION 14			GOING INFRONT OF DOOR NUMBER 1
	{{350,200},NORMAL_SPEED,FORWARD,NULL},		//POSITION 15			GOING INFRONT OF DOOR NUMBER 1 BUT CLOSE
	{{350,80},30,FORWARD,NULL},					//POSITION 16			PUSH DOWN DOOR NUMBER 1
	{{350,400},LOW_SPEED,BACKWARD,NULL},		//POSITION 17			GO BACK AND WAIT FOR OPENING THE UMBRELLA IF THE SERVO WORKS
	{{100,400},LOW_SPEED,BACKWARD,NULL}			//POSITION 18			GO TO SLEEP TO THE LEFT OF THE STARTING AREA
};
const struct goto_fields purple_tactic_five_positions[TACTIC_FIVE_POSITION_COUNT] =
{
	{{85,1220},NORMAL_SPEED,FORWARD,NULL}	
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
	
	for(current_position = next_position;current_position < TACTIC_CAMERA_POSITION; current_position++)
	{
		odometry_status = odometry_move_to_position(&(purple_camera[current_position].point), purple_camera[current_position].speed,
		purple_camera[current_position].direction,purple_camera[current_position].callback);
		if(odometry_status == ODOMETRY_FAIL)
		{
			break;
		}
		else if(current_position == 0)
		{
			//_delay_ms(1000);
		}
		if(current_position == 3)
		{
			int i;
			
			odometry_rotate(80,LOW_SPEED,NULL);                     //rotira se da dodje u poziciju za slikanje kamere
			_delay_ms(2000);
			/*_delay_ms(100);
			for(i=0;i<2;i++)
			{
				if(!(camera()))
				{
					active_state = ROBOT_STATE_TACTIC_ERROR;
				}
				else
				{
					//it is set to gud
				}
			}
			_delay_ms(100);*/
		}
	}//end for
	
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
						_delay_ms(1000);
					}
					if(current_position == 5)
					{
						int i;
						for(i=0;i<59;i++)
						{
							_delay_ms(500);
						}
					}
					else if(current_position == 7)
					{
						_delay_ms(1000);
					}
					else if(current_position == 9)
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
					else if(current_position == 15)
					{
						_delay_ms(1000);
					}
					else if(current_position == 4)
					{
						//servo left grabber up
					}
					else if(current_position == 4 || current_position == 12 || current_position == 14 || current_position == 16)
					{
						_delay_ms(1500);
					}
					else if(current_position == 9)
					{
						_delay_ms(10000);
					}
					else if(current_position == 18)
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
					else if(current_position == 15)
					{
						_delay_ms(1000);
					}
					else if(current_position == 4)
					{
						//servo left grabber up
					}
					else if(current_position == 4 || current_position == 12 || current_position == 14 || current_position == 16)
					{
						_delay_ms(1500);
					}
					else if(current_position == 9)
					{
						_delay_ms(10000);
					}
					else if(current_position == 18)
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
