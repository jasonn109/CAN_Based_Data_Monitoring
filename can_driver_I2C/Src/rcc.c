/*
 * rcc.c
 *
 *  Created on: Sep 12, 2025
 *      Author: JASON PEREIRA
 */

#include "rcc.h"

void config_rcc()
{
	RCC->CR &= ~RCC_CR_HSEBYP; // choose ceramic oscillator
	RCC->CR |= RCC_CR_HSEON; // Enable HSE clock

	while((RCC->CR & RCC_CR_HSERDY)==0);  // wait for HSE clock to lock

	 FLASH->ACR |= FLASH_ACR_PRFTBE;               // enable prefetch buffer
	 FLASH->ACR &= ~FLASH_ACR_LATENCY;             // clear latency bits
	 FLASH->ACR |= FLASH_ACR_LATENCY_1;            // 1 wait state


	RCC->CFGR |= RCC_CFGR_PLLSRC; // set PREDIV as clock source for PLL

	RCC->CFGR &= ~RCC_CFGR_PLLMULL;
	RCC->CFGR |= RCC_CFGR_PLLMULL4; // Mul PLL by 4

	RCC->CFGR |= (RCC_CFGR_PPRE1_DIV2); // APB1 prescaler div by 2
	RCC->CFGR |= (RCC_CFGR_PPRE2_DIV2); // APB2 prescaler div by 2

	RCC->CR |= RCC_CR_PLLON; // Enable PLL

	while((RCC->CR & RCC_CR_PLLRDY)==0); // wait for PLL to lock

	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL; // Choose PLL as the System Clock

	while(!((RCC->CFGR & RCC_CFGR_SWS)== RCC_CFGR_SWS_PLL) );
}
