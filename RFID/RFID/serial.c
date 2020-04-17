#include <avr/io.h>
#include "Serial.h"

void 
uart_init(void)
{	
	UBRR0 = BAUDRATE;
	UCSR0B |= (1<<RXCIE0);
	UCSR0B  |= (1 << TXEN0   ) | (1<< RXEN0 );
	UCSR0C|= (1 << UMSEL01 ) | (1 << UCSZ00) | (1 << UCSZ01);	
}

void 
uart_transmit (char data)
{
	while (!( UCSR0A & (1<<UDRE0)));                // wait while register is free
	UDR0 = data;                                   // load data in the register
}

void 
uart_transmit_string(char *data)
{	
	while (*data != 0)
	{
		uart_transmit(*data++);
	}
}


void
convert_int_to_string(int int_value,char *int_to_ascii)
{
	uint8_t i=0, final_string[10];
	uint8_t g1=0, g2=0;
	do
	{
		final_string[i] = (int_value%10) + 0x30;   /*Converting integer to ascii format*/
		int_value = int_value/10;                  /*Divide the integer value*/
		i++;                            						/*increment the count*/
		}while(int_value!=0);                     /*Loop untill all integers are converted to ascii*/
		
		
		for (g1 = i-1; g1 >= 0; g1--)              /*Reversing the array for original ascii format*/
		{
			int_to_ascii[g2] = final_string [g1];
			g2++;
			if (g1 == 0)
			{
				break;
			}
			int_to_ascii[g2] = '\0';                 /*Placing Null for the last character*/
		}
	}
	
	
void
convert_float_to_string(float float_value, char *float_to_string)
{
	int decimal_value, decimal, i, g1, g2 = 0;
	uint8_t result[20];
	decimal_value = float_value;
	
	decimal = (int)(float_value * 1000) % 1000;
	
	result[0] = (decimal % 10) + '0';
	result[1] = ((decimal /= 10) % 10) + '0';
	result[2] = (decimal / 10) + '0';
	result[3] = '.';
	i = 4;
	
	while (decimal_value > 0)
	{
		result[i] = (decimal_value % 10) + '0';
		decimal_value /= 10;
		i++;
	}
	
	for (g1 = i-1; g1 >= 0; g1--)              /*Reversing the array for original ascii format*/
	{
		float_to_string[g2] = result [g1];
		g2++;
		if (g1 == 0)
		{
			break;
		}
		float_to_string[g2] = '\0';                 /*Placing Null for the last character*/
	}
}
	
char 
uart_recieve(void)
{	
	while (!((UCSR0A) & (1<<RXC0)));	
	return UDR0;	
}

/*ISR(USART_RX_vect)
{	
	unsigned char rchar = uart_recieve();
	uart_transmit(rchar);	
}*/