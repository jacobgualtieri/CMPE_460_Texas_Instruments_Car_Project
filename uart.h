#ifndef _UART_
#define _UART_

#include "Common.h"
void uart0_init(void);
BYTE uart0_getchar(void);
void uart0_putchar(char ch);
void uart0_put(char *ptr_str);

//add uart2 functionality
void uart2_init(void);
BYTE uart2_getchar(void);
void uart2_get(char *string_buffer, int len);
void uart2_putchar(char ch);
void uart2_put(char *ptr_str);

//add a function that "peeks" at the uart
//to see if a character is available to be read
BOOLEAN uart0_dataAvailable(void);
BOOLEAN uart2_dataAvailable(void);

#endif
