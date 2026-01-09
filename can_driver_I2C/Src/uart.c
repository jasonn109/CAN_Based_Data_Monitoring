/*
 * uart.c
 *
 *  Created on: Sep 6, 2025
 *      Author: JASON PEREIRA
 */


#include "uart.h"
static uint16_t compute_uart_div(uint32_t pclk, uint32_t buadrate );
static void set_baudrate(USART_TypeDef* USARTx, uint32_t pclk, uint32_t buadrate);

void uart1_init()
{
// enable APB2 clock access to GPIOA
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

/* Configure PA9 as Alternate fucntion push pull and 50MHz output(MAX) pin*/
	GPIOA->CRH |= (0x3<<4); // MODE[1:0] = 11
	GPIOA->CRH &= ~(0x1<<6); // CNF[0] = 0
	GPIOA->CRH |= (0x1<<7); // CNF[1] = 1

/* Alternate function remap = 0*/
	AFIO->MAPR &= ~(AFIO_MAPR_USART1_REMAP);

/* enable APB2 clock access to USART1 module*/
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

/*Configure the required Baudrate */
	set_baudrate(USART1,PCLK1,Baudrate );

/* Set the transfer direction */
	USART1->CR1 |= USART_CR1_TE;

/* Enable the uart module */
	USART1->CR1 |= USART_CR1_UE;


}

static uint16_t compute_uart_div(uint32_t pclk, uint32_t buadrate)
{
	return ((pclk +(buadrate/2U))/buadrate);
}

//static uint16_t compute_uart_div_1(uint32_t pclk, uint32_t buadrate)
//{
//	uint32_t div_mantissa = (pclk / buadrate*16U);
//
//}


static void set_baudrate(USART_TypeDef* USARTx, uint32_t pclk, uint32_t buadrate)
{
	USARTx->BRR = compute_uart_div(pclk,buadrate);
}


void uart_write(USART_TypeDef * USARTx, uint8_t data)
{
	/* wait until Transmit data reg is empty*/
	while((USARTx->SR & USART_SR_TXE)==0){}

	/* write to the DR */
	USARTx->DR = (0xFF & data);
}

int __io_putchar(int ch)
{
	uart_write(USART1, ch);
	return ch;
}
