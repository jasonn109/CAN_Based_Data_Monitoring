/*
 * i2c_dma.h
 *
 *  Created on: Oct 18, 2025
 *      Author: JASON PEREIRA
 */

#ifndef I2C_DMA_H_
#define I2C_DMA_H_
#include "stm32f4xx.h"
#include "stm32f446xx.h"
#include "stddef.h"


void I2C1_Init(void);
void i2c1_init(void);
void dma1_stream6_i2c1_tx_init(void);
void dma1_stream6_i2c1_transfer(uint8_t *msg_to_send, uint32_t msg_len);
void i2c_dma_write(uint8_t slave_addr, uint8_t *p_write_buff, uint16_t num_of_bytes);
void I2C1_burstWrite(uint8_t saddr, uint8_t maddr, uint16_t n, uint8_t* data);
#endif /* I2C_DMA_H_ */
