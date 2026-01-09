/*
 * rcc.h
 *
 *  Created on: Sep 12, 2025
 *      Author: JASON PEREIRA
 */

#ifndef RCC_H_
#define RCC_H_
#include "stm32f1xx.h"
#include "stm32f103xb.h"
void config_rcc();


#define SYSTEM_CLK	32000000
#define PCLK1		SYSTEM_CLK/2u
#define PCLK2		SYSTEM_CLK/2u




#endif /* RCC_H_ */
