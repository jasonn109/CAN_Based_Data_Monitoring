/*
 * SysTick_driver.h
 *
 *  Created on: Oct 12, 2025
 *      Author: JASON PEREIRA
 */

#ifndef SYSTICK_DRIVER_H_
#define SYSTICK_DRIVER_H_
#include "stm32f1xx.h"
#include "stm32f103xb.h"
#include "rcc.h"


#define SYSTICK_LOAD_VAL			32000-1
#define	CTRL_ENABLE					(1U<<0)
#define CTRL_CLKSRC					(1U<<2)
#define CTRL_COUNTFLAG				(1U<<16)


void systickDelayMs(int delay);

#endif /* SYSTICK_DRIVER_H_ */
