/*
 * rcc.h
 *
 *  Created on: Oct 18, 2025
 *      Author: JASON PEREIRA
 */
/*
 * RCC_clock.h
 *
 *  Created on: Oct 17, 2025
 *      Author: JASON PEREIRA
 */

/*
 * SysClockConfig.h
 *
 *  Created on: Aug 10, 2024
 *      Author: faruk
 */

#ifndef INC_RCC_CLOCK_H_
#define INC_RCC_CLOCK_H_
#include "stdint.h"
void SysClockConfig (void);
void SysTick_Init(void);
void delay_ms(uint32_t ms);

#endif /* INC_RCC_CLOCK_H_ */
