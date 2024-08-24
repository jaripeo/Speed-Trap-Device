#include "encoder.h"
#include <avr/eeprom.h>

void encoder_init(void)
{
	if (!b && !a){
		old_state = 0;
	}
    else if (!b && a){
		old_state = 1;
	}
    else if (b && !a){
		old_state = 2;
	}
    else{
		old_state = 3;
	}

    new_state = old_state;

	//Read the speed threshold from EEPROM, if it is out of bounds, set it to 15
	speedThreshold = eeprom_read_byte((void *) 1);
	if (speedThreshold > 99) {
    	speedThreshold = 15;
	} 
	else if (speedThreshold < 1) {
    	speedThreshold = 15;
	}
}

void handle_rotary_encoder(void)
{
	char x = PIND;
	a = x & (1 << PD3);
	b = x & (1 << PD2);

	if (old_state == 0) {

		// Handle A and B inputs for state 0
		if(!a && b)
		{
			new_state = 1;
			speedThreshold--;
		}
		else if(a && !b)
		{
			new_state = 3;
			speedThreshold++;
		}

	}
	else if (old_state == 1) {

		// Handle A and B inputs for state 1
		if(a && b)
		{
			new_state = 2;
			speedThreshold--;
		}
		else if(!a && !b)
		{
			new_state = 0;
			speedThreshold++;
		}

	}
	else if (old_state == 2) {

		// Handle A and B inputs for state 2
		if(a && !b)
		{
			new_state = 3;
			speedThreshold--;
		}
		else if(!a && b)
		{
			new_state = 1;
			speedThreshold++;
		}

	}
	else {   // old_state = 3

		// Handle A and B inputs for state 3
		if(!a && !b)
		{
			new_state = 0;
			speedThreshold--;
		}
		else if(a && b)
		{
			new_state = 2;
			speedThreshold++;
		}
	}

	if (speedThreshold > 99) {
    	speedThreshold = 99;
	} 
	else if (speedThreshold < 1) {
    	speedThreshold = 1;
	}	

	// If state changed, update the value of old_state,
	// and set a flag that the state has changed.
	if (new_state != old_state) 
	{
		old_state = new_state;
		eeprom_update_byte((void *) 1, speedThreshold);
	}
}
