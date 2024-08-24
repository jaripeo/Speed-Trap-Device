/********************************************
 *
 *  Name: Dave Rodriguez
 *  Email: daverodr@usc.edu
 *  Section: Wed @3:30
 *  Assignment: Lab 10 - Final project
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include "lcd.h"
#include "encoder.h"
#include "timers.h"
#include "serial.h"

void play_note(uint16_t);

#define TIMER1_THRESHOLD 62500 //(16,000,000 * 4) / 1024
#define PRESCALAR 1024 
#define F_CPU 16000000
#define BUFFER_SIZE 5


//checkpoint 1 variables
volatile uint16_t timerValue = 0;
volatile uint16_t timing_expired = 0;
volatile uint32_t elapsedTime = 0;
volatile uint32_t speed = 0;
volatile uint32_t DISTANCE_CM = 38100; //1.5 inches to cm

//checkpoint 2 variables
 volatile uint32_t speedThreshold = 1;
//volatile uint8_t unsigned char speedThreshold = 1;
uint8_t volatile a, b;
int volatile isBuzzing = 0;  // Flag for state change
uint8_t volatile new_state, old_state;
int volatile interrupts = 0;
uint16_t volatile freq1 = 440;

//checkpoint 3 variables
volatile char buffer2[BUFFER_SIZE];
volatile uint8_t data_started;
volatile uint8_t buffer_count;
volatile uint8_t data_valid;

int main(void) {
	// Initialize the LCD
	lcd_init();

    // Initialize DDR and PORT registers and LCD
	DDRB |= (1 << PB3);	// Set port B bit 3 for output for the servo
	DDRB |= (1 << PB4); // Set port B bit 4 for output for the Tri-state buffer enable
	DDRB |= (1 << PB5); // LED for indicating timing in progress 

	DDRC |= (1 << PC1); // Set port C bit 1 for output for the buzzer
	DDRC |= (1 << PC2); // Set port C bit 2 for output for the red segment of the RGB LED
	DDRC |= (1 << PC3); // Set port C bit 3 for output for the blue segment of the RGB LED

	//turn red and blue LEDs off
	PORTC |= (1 << PC2);
	PORTC |= (1 << PC3);

	//Start sensor in PORTC, bit 4 (PC4)
	//Stop sensor in PORTC, bit 5 (PC5)
	//RS-232 Serial Rx input in PORTD, bit 0 (PD0)
	//RS-232 Serial Tx output in PORTD, bit 1 (PD1)
	//Rotary encoder B input in PORTD, bit 2 (PD2)
	//Rotary encoder A input in PORTD, bit 3 (PD3)
	PORTD |= (1 << PD2) | (1 << PD3); // Enable pull-ups

	PCMSK1 |= (1 << PCINT12 | 1 << PCINT13); // Enable PCINT12 and PCINT13 for PC4 and PC5.
	PCMSK2 |= (1 << PCINT18 | 1 << PCINT19); // Enable PCINT18 and PCINT19 for PD2 and PD3.
	PCICR |= (1 << PCIE1); // Enable Pin Change 1 to enable PCINT8-14
	PCICR |= (1 << PCIE2); // Enable Pin Change 2 to enable PCINT16-23
	

	timer1_init();
	timer2_init();
	serial_init();
	sei(); // Enable global interrupts

	// Write splash screen
	char first[17];
    snprintf(first, 17, " Dave Rodriguez ");

    lcd_moveto(0,0);
    lcd_stringout(first);

    char second[16];
    snprintf(second, 16, "Section: 31292R");

    lcd_moveto(1,0);
    lcd_stringout(second);

    _delay_ms(2000);

    lcd_writecommand(1);        // Clear the screen

	encoder_init();

    while (1) {
		OCR2A = (3500 - (23*(speed/10))) / 100; //for the servo motor

		if(timing_expired == 0)
		{
			char buffer[17];
			sprintf(buffer, "%3dms = ", elapsedTime);
			lcd_moveto(0,0);
			lcd_stringout(buffer);
			
			sprintf(buffer, "%3d.", speed / 10);
			lcd_moveto(0,9);
			lcd_stringout(buffer);

			sprintf(buffer, "%1d  ", speed % 10);
			lcd_moveto(0,13);
			lcd_stringout(buffer);

			//second line of LCD
			char buffer1[17];
			sprintf(buffer1, "Max=%2d   ", speedThreshold);
			lcd_moveto(1,0);
			lcd_stringout(buffer1);

			if(speed/10 > speedThreshold && !isBuzzing)
			{
				timer0_init();
				play_note(freq1);
				isBuzzing = 1;
			}

			if(data_valid) // ‘}’ has been received and the buffer contains a valid speed string
			{
				int incomingSpeed = 0;
				sscanf(buffer2, "%3d", &incomingSpeed); //parse the speed from the buffer

				sprintf(buffer1, "%3d.", incomingSpeed/10);
				lcd_moveto(1,9);
				lcd_stringout(buffer1);

				sprintf(buffer1, "%d  ", incomingSpeed % 10);
				lcd_moveto(1,13);
				lcd_stringout(buffer1);

				if(abs(speed - incomingSpeed) > 30)
				{
					if(incomingSpeed > speed)
					{
						//turn on the red LED if remote speed is more than 3cm/sec or higher than local speed
						PORTC &= ~(1 << PC2);
						PORTC |= (1 << PC3);
					}
					else
					{
						//turn on the blue LED if remote speed is less than 3cm/sec or lower than local speed
						PORTC |= (1 << PC2);
						PORTC &= ~(1 << PC3);
					}
				}
				else
				{
					//if the speeds are within 3cm/sec of each other, or no local/remote speeds, turn off the LEDs
					PORTC |= (1 << PC2);
					PORTC |= (1 << PC3);
				}
			}
		}

		else
		{
			char msg[17];
    		snprintf(msg, 17, "Time expired    ");

    		lcd_moveto(1,0);
    		lcd_stringout(msg);
		}
    }
}


ISR(PCINT1_vect)
{
	if(!(PINC & (1 << PC5))) //stop sensor is actuated
	{
		isBuzzing = 0;
		//turn off the LED to indicate timing is complete
		PORTB &= ~(1 << PB5);
		//stop the timer
		TCCR1B &= ~(1 << CS10) & ~(1 << CS12);
		//retrieve the timer's count value
		timerValue = TCNT1; 
		//calculate the elapsed time in milliseconds
		elapsedTime = (uint32_t)timerValue * PRESCALAR / 16000;
		//calculate the speed in cm/s
		speed = DISTANCE_CM / elapsedTime; //38100 cm / elapsedTime ms

		char speedBuffer[6];
		sprintf(speedBuffer, "%d", speed);

		//transmit the start character
		tx_char('{');

		//transmit the speed
		int i;
		for(i = 0; speedBuffer[i] != '\0'; i++)
		{
			tx_char(speedBuffer[i]);
		}

		//transmit the end character
		tx_char('}');
    }
	//If start sensor is actuated
	else if(!(PINC & (1 << PC4)))
	{
		//turn on the LED to indicate timing in progress
		PORTB |= (1 << PB5);
		//Clear the timer's count register to zero
		TCNT1 = 0;
		//set the prescalar to 1024 and start the timer
		TCCR1B |= (1 << CS10) | (1 << CS12);
		//set the timing expired flag
		timing_expired = 0;
	}
}

//used to handle the rotary encoder
ISR(PCINT2_vect)
{
    handle_rotary_encoder();
}

//used to measure the speed 
ISR(TIMER1_COMPA_vect)
{
	//stop the timer
	TCCR1B &= ~(1 << CS10) & ~(1 << CS12);
	//set the timing expired flag
	timing_expired = 1;
	PORTB &= ~(1 << PB5);
}



//buzzer functions
void play_note(uint16_t freq)
{
    OCR0A = F_CPU / (2 * freq); // set the period
    TCCR0B |= (1 << CS12); // set the prescaler to 256
}


ISR(TIMER0_COMPA_vect) {
    PORTC ^= (1 << PC1);
	
	interrupts++;

	//If the ISR has run enough times to generate the tone for one second, turn off the timer.
	if(interrupts == freq1/4)
	{
		TCCR0B &= ~(1 << CS12); //stop the timer
		interrupts = 0;
	}
}


//serial ISR
ISR(USART_RX_vect)
{
	rx_char();
}


