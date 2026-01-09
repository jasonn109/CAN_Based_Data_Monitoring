/*
 * adxl345.h
 *
 *  Created on: Oct 17, 2025
 *      Author: vivian
 */

#ifndef ADXL345_H_
#define ADXL345_H_

#include "i2c.h"
#include <stdint.h>

#define DEVID_R             (0x00)
#define DEVICE_ADDR         (0x53)
#define DATA_FORMAT_R       (0x31)
#define POWER_CTL_R         (0x2D)
#define DATA_START_ADDR     (0x32)

#define FOUR_G              (0X01) /*To write into data reg to config +/- 4G*/
#define RESET               (0x00) /*To reset the power control reg*/
#define SET_MEASURE_B       (0x08) /*To set the measure and wake up at 8hz*/


void adxl_init (void);
void adxl_read_values (uint8_t reg);



#endif /* ADXL345_H_ */
