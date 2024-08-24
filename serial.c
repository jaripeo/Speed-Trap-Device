#include "serial.h"

void serial_init(void)
{
    UBRR0 = MYUBRR;
    UCSR0B |= (1 << TXEN0 | 1 << RXEN0); // Enable RX and TX
    UCSR0C = (3 << UCSZ00); // Async., no parity,
                            // 1 stop bit, 8 data bits
    UCSR0B |= (1 << RXCIE0); // Enable RX interrupt
}

char rx_char()
{
    char received_char = UDR0; // Read the received character

    if (received_char == '{') 
    {
        data_started = 1;
        buffer_count = 0;
        data_valid = 0;
        buffer2[0] = '\0';
    } 
    else if (data_started) 
    {
        if (received_char >= '0' && received_char <= '9' && buffer_count < BUFFER_SIZE - 1) 
        {
            buffer2[buffer_count++] = received_char;
        } 
        else if (received_char == '}' && buffer_count > 0) 
        {
            buffer2[buffer_count] = '\0'; // Null-terminate the string
            data_valid = 1;
        } 
        else 
        {
            data_started = 0;
        }
    }
}

void tx_char(char ch)
{
    // Wait for transmitter data register empty
    while ((UCSR0A & (1<<UDRE0)) == 0) {}
    UDR0 = ch;
}