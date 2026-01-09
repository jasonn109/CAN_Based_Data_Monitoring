/*
 * uart.h
 *
 *  Created on: Sep 6, 2025
 *      Author: JASON PEREIRA
 */

#ifndef UART_H_
#define UART_H_
#include "stm32f1xx.h"
#include "stm32f103xb.h"
#include "stdio.h"
#include "rcc.h"


#define Baudrate	9600


int __io_putchar(int ch);

void uart1_init();

void uart_write(USART_TypeDef * USARTx, uint8_t data);

#endif /* UART_H_ */
