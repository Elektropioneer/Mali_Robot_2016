#include "gpio.h"
#include "system.h"
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
	/*timer_init(1000);
	timer_register_callback(gpio_debouncer);
	gpio_register_pin(56, GPIO_DIRECTION_OUTPUT, false);
	gpio_register_pin(57, GPIO_DIRECTION_OUTPUT, false);
	gpio_register_pin(58, GPIO_DIRECTION_OUTPUT, false);
	gpio_register_pin(48, GPIO_DIRECTION_OUTPUT, false);
	gpio_register_pin(19, GPIO_DIRECTION_INPUT, false);
	
	
	uint8_t val = 0;*/
	system_init();
	
	//this is for testing the callback in greenside first position
	gpio_register_pin(48, GPIO_DIRECTION_OUTPUT,false);
    while (1) 
    {
		greenside();
		//PORTG = (gpio_read_pin(19) == 0) ? 0 : 0Xff;
			
		//gpio_write_pin(48, gpio_read_pin(19));
    }
}

