/*
 * candriver.h
 *
 *  Created on: Sep 5, 2025
 *      Author: JASON PEREIRA
 */

#ifndef CANDRIVER_H_
#define CANDRIVER_H_


#include "stm32f4xx.h"
#include "stm32f446xx.h"



typedef struct
{
	uint32_t std_id;
	uint32_t ext_id;
	uint32_t ide;
	uint32_t rtr;
	uint32_t dlc;
	uint8_t transmit_global_time;

}can_tx_header_typedef;


typedef struct
{
  uint32_t std_id;    /* Specifies the standard identifier.*/


  uint32_t ext_id;    /* Specifies the extended identifier.*/


  uint32_t ide;      /* Specifies the type of identifier for the message that will be transmitted.*/


  uint32_t rtr;      /* Specifies the type of frame for the message that will be transmitted.*/


  uint32_t dlc;      /* Specifies the length of the frame that will be transmitted.*/


  uint32_t timestamp; /* Specifies the timestamp counter value captured on start of frame reception.*/


  uint32_t filter_match_index; /*!< Specifies the index of matching acceptance filter element.*/


} can_rx_header_typedef;

#define CAN_MODE_LOOPBACK  0x00
#define CAN_MODE_NORMAL  0x01

#define CAN_ID_STD		0x00
#define CAN_ID_EXT		0x04

#define CAN_RX_FIFO0	0x00
#define CAN_RX_FIFO1	0x01
void can_gpio_init(void);
void can_params_init(uint8_t mode);
void can_start(void);
uint8_t can_add_tx_message(can_tx_header_typedef *pHeader, uint8_t aData[], uint32_t *pTxMailbox);
uint8_t can_get_rx_message(uint32_t RxFifo, can_rx_header_typedef *pHeader, uint8_t aData[]);
void can_filter_config(uint16_t std_id);





#endif /* CANDRIVER_H_ */
