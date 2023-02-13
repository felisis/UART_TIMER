#include <stdio.h>
#include <stdarg.h>
#include "main.h"

#define DEBUGMSG_SIZE   128
/// char printMsg[DEBUGMSG_SIZE];       // use local memory

USART_TypeDef* pdbgUart = USART2;

//UART_ST Uart_DBG;

int printf_putchar(int ch)
{
	while (!(pdbgUart->SR & USART_SR_TXE));
	pdbgUart->DR = (char)ch;
	return(ch);
}

void PRINTF_DEBUG(char* fmt, ...)  // PRINTF_DEBUG("Hello world....");
{
	char printMsg[DEBUGMSG_SIZE];
	char* pbuf;
	va_list ap;
	va_start(ap, fmt);
	vsnprintf((char*)printMsg, DEBUGMSG_SIZE, fmt, ap);
	va_end(ap);

	pbuf = printMsg;
	while (*pbuf != 0) {
		printf_putchar(*pbuf++);
	}
}

