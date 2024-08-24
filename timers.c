#include "timers.h"

void timer1_init()
{
	//set the output compare register to the threshold value
	OCR1A = TIMER1_THRESHOLD;
	//enable TIMER1 compare match A interrupt
	TIMSK1 |= (1 << OCIE1A);
}

void timer2_init(void)
{
	TCCR2A |= (0b11 << WGM20);  // Fast PWM mode, modulus = 256
	TCCR2A |= (0b10 << COM2A0); // Turn PB3 on at 0x00 and off at OCR2A
	OCR2A = (3500 - (23*(speed/10))) / 100; //Linear equation
	TCCR2B |= (0b111 << CS20);  // Prescaler = 1024 for 16ms period
}

void timer0_init() {
    TCCR0A |= (1 << WGM12); // set the mode to CTC
    TIMSK0 |= (1 << OCIE0A); // enable the interrupt
}