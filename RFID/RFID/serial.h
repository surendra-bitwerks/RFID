#ifndef __SERIAL_H__
#define __SERIAL_H__

#define F_CPU 16000000UL
#define BAUD 9600
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)


void uart_init(void);
void uart_transmit (char data);
void uart_transmit_string(char *data);
char uart_recieve(void);
void convert_int_to_string(int int_value, char *int_to_ascii);
void convert_float_to_string(float float_value, char *float_to_string);
#endif