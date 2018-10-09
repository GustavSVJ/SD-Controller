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
void uart_putc(uint8_t c);
uint8_t uart_getc();
void init_usb_uart(uint32_t baud);


#endif /* UART_H*/
