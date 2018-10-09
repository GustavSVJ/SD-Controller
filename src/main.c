#include "stm32f30x_conf.h" // STM32 config
#include "Uart.h"
#include "lcd.h"
#include "flash.h"
#include <stdio.h>
#include <string.h>

int main(void){


    uart2_init(9600);
    uart1_init(9600);

    for (uint32_t i = 0; i < 0xfffff; i++);

    uint8_t str[50];
    memset(str, 0, 50);

    while(1){
        uart2_putc('x');
        sprintf(str, "Hello again world\n");
        uart1_putstr(str);
        for (uint32_t i = 0; i < 0xfffff; i++);
    }

    return 0;
}
