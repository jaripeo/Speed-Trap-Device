#include <avr/io.h>

#define FOSC 16000000 // Clock Speed
#define BAUD 9600 // Baud Rate
#define MYUBRR (FOSC/16/BAUD-1) // Value for UBRR0 register
#define BUFFER_SIZE 5

extern volatile char buffer2[BUFFER_SIZE];
extern volatile uint8_t data_started;
extern volatile uint8_t buffer_count;
extern volatile uint8_t data_valid;

void serial_init(void);
char rx_char();
void tx_char(char ch);