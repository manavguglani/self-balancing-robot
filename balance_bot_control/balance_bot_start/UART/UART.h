#ifndef UART_H_
#define UART_H_

#define RX  (1<<4)
#define TX  (1<<3)
#define TE  (1<<5)
#define RE  (1<<7)

void uart0_init();
void uart_tx(char data);
char uart_rx();
void uart_tx_string(char *data);
void uart_tx_array(const char *array,int arr_length);
int uart_read();

#endif 