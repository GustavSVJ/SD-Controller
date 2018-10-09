#ifndef _UART_H_
#define _UART_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f30x_conf.h"
#include <stdio.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */
/****************************/
/*** USB Serial Functions ***/
/****************************/
void uart2_putc(uint8_t c);
uint8_t uart2_getc();
void uart2_init(uint32_t baud);

void uart1_putc(uint8_t c);
uint8_t uart1_getc();
void uart1_init(uint32_t baud);



#endif /* UART_H*/
