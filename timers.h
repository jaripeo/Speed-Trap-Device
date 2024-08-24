#include <avr/io.h>

#define TIMER1_THRESHOLD 62500 //(16,000,000 * 4) / 1024
extern volatile uint32_t speed;

void timer1_init(void);
void timer2_init(void);
void timer0_init(void);
