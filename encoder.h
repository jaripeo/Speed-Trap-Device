#include <avr/io.h>

extern volatile uint8_t a, b;
extern volatile uint8_t new_state, old_state;
extern volatile uint32_t speedThreshold;

void handle_rotary_encoder(void);
void encoder_init(void);
