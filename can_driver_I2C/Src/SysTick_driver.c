/*
 * SysTick_driver.c
 *
 *  Created on: Oct 12, 2025
 *      Author: JASON PEREIRA
 */


#include "SysTick_driver.h"

void systickDelayMs(int delay)
{
	/*Reload with number of clocks per millisecond*/
	SysTick->LOAD	= SYSTICK_LOAD_VAL;

	/*Clear systick current value register */
	SysTick->VAL = 0;

	/*Enable systick and select internal clk src*/
	SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC;


	for(int i=0; i<delay ; i++){

		/*Wait until the COUNTFLAG is set*/

		while((SysTick->CTRL &  CTRL_COUNTFLAG) == 0){}

	}

	SysTick->CTRL = 0;

}
