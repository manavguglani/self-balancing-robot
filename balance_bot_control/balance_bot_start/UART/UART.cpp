/*********************************UART_LIBRARY****************************/
#include <avr/io.h>
#include "UART.h"
/**********************************
function name	:	uart0_init
functionality	:	initializes uart0, sets baud rate, enables reciever and transmit,setting data frame
arguments		:	none
return value	:	void
**********************************/
void uart0_init()
{
	UCSR0B = 0x00;							//disable while setting baud rate

	UBRR0L = 95; 							// for the clock frequency 14745600 and the baud rate 9600, value of UBRR is 95
	UBRR0H = 0x00;

	UCSR0C = 3<<1;							//setting 8-bit character and 1 stop bit
	UCSR0B = RX | TX;						//enabling receiver and transmit
}

/**********************************
function name	:	uart_tx
functionality	:	sends a character
arguments		:	a character (data)	which needs to be send
return value	:	void
***********************************/
void uart_tx(char data)
{
	while(!(UCSR0A & TE));						//waiting to transmit
	UDR0 = data;
}

/***********************************
function name	:	uart_tx_array
functionality	:	sends an entire array 
arguments		:	the array which needs to be send and the array length
return value	:	void
***********************************/
void uart_tx_array(const char *array,int arr_length)
{
	int i=0;
	while(i < arr_length)
	{
		uart_tx(array[i++]);
	}
}

/************************************
function name	:	uart_tx_string
functionality	:	sends a string
arguments		:	the address of the string
return value	:	void
************************************/
void uart_tx_string(char *data)
{
	while(*data != '\0')
	{
		uart_tx(*data);
		data++;
	}
}

/*************************************
function name	:	uart_rx
functionality	:	recieve a character (it waits unless a character is recieved)
arguments		:	none
return value	:	the recieved character
*************************************/
char uart_rx()
{
	while(!(UCSR0A & RE));						//waiting to receive
	return UDR0;
}

/*************************************
function_name	:	uart_read
functionality	:	recieve a character (it does not wait for a character)
arguments		:	none
return value	:	returns the character is pressed else returns -1
**************************************/
int uart_read()
{
	if(UCSR0A & RE)
		return UDR0;
	else
		return -1;
}