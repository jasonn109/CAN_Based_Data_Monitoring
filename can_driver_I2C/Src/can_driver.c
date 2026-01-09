/*
 * candriver.c
 *
 *  Created on: Sep 5, 2025
 *      Author: JASON PEREIRA
 */


#include "can_driver.h"



void can_gpio_init(void)
{
	// enable APB2 clock access to GPIOA
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	//enable APB2 clock access to Alternate function IO
		RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	/* Configure PA11 CAN Rx as Input Floating */
		GPIOA->CRH &= ~(0x3<<12); // MODE[1:0] = 00
		GPIOA->CRH |= (0x1<<14); // CNF[0] = 1
		GPIOA->CRH &= ~(0x1<<15); // CNF[1] = 0

		//GPIOA->ODR |= (0x1<<11); // set pin PA11 to PULL UP

	/* Configure PA12 CAN Tx Alternate function push pull and 50MHz output(MAX) pin*/
		GPIOA->CRH |= (0x3<<16); // MODE[1:0] = 11
		GPIOA->CRH &= ~(0x1<<18); // CNF[0] = 0
		GPIOA->CRH |= (0x1<<19); // CNF[1] = 1


	/* Alternate function remap = 0*/
		AFIO->MAPR &= ~(0x3<<13);

	/* Enable CAN Rx0 interrupt in the NVIC for message reception*/
		NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);


}

void can_params_init(uint8_t mode){

	/*Enable clock access to CAN1*/
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

	/*Enter initialization mode*/
	CAN1->MCR |= CAN_MCR_INRQ;

	/*Wait until CAN1 is in initialization mode*/
	while((CAN1->MSR & CAN_MSR_INAK) == 0){}

	/*Exit sleep mode*/
	CAN1->MCR &=~CAN_MCR_SLEEP;

	/*Wait until CAN1 is out of sleep mode*/
	while((CAN1->MSR & CAN_MSR_SLAK) != 0){}

	/*Configure timing parameters including baudrate by configuring time segment 1 and 2
	 * and prescaler*/
	/* Generates a Buadrate of 500kbps*/
	CAN1->BTR = (3<< CAN_BTR_TS2_Pos) | (2 << CAN_BTR_TS1_Pos) | (3 << CAN_BTR_BRP_Pos);
	//CAN1->BTR = (1<< CAN_BTR_TS1_Pos) | (0 << CAN_BTR_TS1_Pos) | (9 << CAN_BTR_BRP_Pos);
	/*Select mode*/
	if(mode)
	{
		/*Normal mode*/
		CAN1->BTR &=~(1U<<30);
	}
	else
	{
		/*Loopback mode*/
		CAN1->BTR |= (1U<<30);
	}
}


void can_start(void)
{
	/*Exit initialization mode*/
	CAN1->MCR &= ~CAN_MCR_INRQ;

	/*Wait until CAN1 is out of initialization mode*/
	while((CAN1->MSR & CAN_MSR_INAK) == 1){}

	/*Enable interrupt for FIFO0 message pending*/
	CAN1->IER |= CAN_IER_FMPIE0;
	/*Enable interrupt for FIFO1 message pending*/
	//CAN1->IER |= CAN_IER_FMPIE1;
}



uint8_t can_add_tx_message(can_tx_header_typedef *pHeader, uint8_t aData[], uint32_t *pTxMailbox)
{
    // Variable to hold the selected transmit mailbox

  uint32_t transmitmailbox;

  // Read the Transmit Status Register

  uint32_t tsr = READ_REG(CAN1->TSR);

  // Check that at least one Tx mailbox is empty
    if (((tsr & CAN_TSR_TME0) != 0U) ||
        ((tsr & CAN_TSR_TME1) != 0U) ||
        ((tsr & CAN_TSR_TME2) != 0U))
    {
      /* Select an empty transmit mailbox */
      transmitmailbox = (tsr & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;

      /* Check transmitmailbox validity */
      if (transmitmailbox > 2U)
      {
        return 1; // Invalid mailbox selected
      }

      /* Store the Tx mailbox */
      *pTxMailbox = (uint32_t)1 << transmitmailbox;

      /* Set up the Id */
      if (pHeader->ide == CAN_ID_STD)
      {
       CAN1->sTxMailBox[transmitmailbox].TIR = ((pHeader->std_id << CAN_TI0R_STID_Pos) |
                                                           pHeader->rtr);
      }
      else
      {
        CAN1->sTxMailBox[transmitmailbox].TIR = ((pHeader->ext_id << CAN_TI0R_EXID_Pos) |
                                                           pHeader->ide |
                                                           pHeader->rtr);
      }

      /* Set up the DLC */
     CAN1->sTxMailBox[transmitmailbox].TDTR = (pHeader->dlc);

      /* Set up the Transmit Global Time mode */
      if (pHeader->transmit_global_time == 1)
      {
        SET_BIT(CAN1->sTxMailBox[transmitmailbox].TDTR, CAN_TDT0R_TGT);
      }

      /* Set up the data field */
      WRITE_REG(CAN1->sTxMailBox[transmitmailbox].TDHR,
                ((uint32_t)aData[7] << CAN_TDH0R_DATA7_Pos) |
                ((uint32_t)aData[6] << CAN_TDH0R_DATA6_Pos) |
                ((uint32_t)aData[5] << CAN_TDH0R_DATA5_Pos) |
                ((uint32_t)aData[4] << CAN_TDH0R_DATA4_Pos));

      WRITE_REG(CAN1->sTxMailBox[transmitmailbox].TDLR,
                ((uint32_t)aData[3] << CAN_TDL0R_DATA3_Pos) |
                ((uint32_t)aData[2] << CAN_TDL0R_DATA2_Pos) |
                ((uint32_t)aData[1] << CAN_TDL0R_DATA1_Pos) |
                ((uint32_t)aData[0] << CAN_TDL0R_DATA0_Pos));

      /* Request transmission */
      SET_BIT(CAN1->sTxMailBox[transmitmailbox].TIR, CAN_TI0R_TXRQ);

      /* Return function status */
      return 0;
    }
    return 0;
}


uint8_t can_get_rx_message(uint32_t RxFifo, can_rx_header_typedef *pHeader, uint8_t aData[])
{

    /* Check the Rx FIFO */
    if (RxFifo == CAN_RX_FIFO0) /* Rx element is assigned to Rx FIFO 0 */
    {
      /* Check that the Rx FIFO 0 is not empty */
      if ((CAN1->RF0R & CAN_RF0R_FMP0) == 0U)
      {
        return 1;
      }
    }
    else /* Rx element is assigned to Rx FIFO 1 */
    {
      /* Check that the Rx FIFO 1 is not empty */
      if ((CAN1->RF1R & CAN_RF1R_FMP1) == 0U)
      {
        return 1;
      }
    }

    // Read the header information from the FIFO mailbox
     // Extract identifier, DLC, timestamp, etc.

    pHeader->ide = CAN_RI0R_IDE & CAN1->sFIFOMailBox[RxFifo].RIR;
    if (pHeader->ide == CAN_ID_STD)
    {
      pHeader->std_id = (CAN_RI0R_STID & CAN1->sFIFOMailBox[RxFifo].RIR) >> CAN_TI0R_STID_Pos;
    }
    else
    {
      pHeader->ext_id = ((CAN_RI0R_EXID | CAN_RI0R_STID) & CAN1->sFIFOMailBox[RxFifo].RIR) >> CAN_RI0R_EXID_Pos;
    }
    pHeader->rtr = (CAN_RI0R_RTR & CAN1->sFIFOMailBox[RxFifo].RIR);
    pHeader->dlc = (CAN_RDT0R_DLC & CAN1->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_DLC_Pos;
    pHeader->filter_match_index = (CAN_RDT0R_FMI & CAN1->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_FMI_Pos;
    pHeader->timestamp = (CAN_RDT0R_TIME & CAN1->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_TIME_Pos;

    // Extract the data from the FIFO mailbox
    aData[0] = (uint8_t)((CAN_RDL0R_DATA0 & CAN1->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA0_Pos);
    aData[1] = (uint8_t)((CAN_RDL0R_DATA1 & CAN1->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA1_Pos);
    aData[2] = (uint8_t)((CAN_RDL0R_DATA2 & CAN1->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA2_Pos);
    aData[3] = (uint8_t)((CAN_RDL0R_DATA3 & CAN1->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA3_Pos);
    aData[4] = (uint8_t)((CAN_RDH0R_DATA4 & CAN1->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA4_Pos);
    aData[5] = (uint8_t)((CAN_RDH0R_DATA5 & CAN1->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA5_Pos);
    aData[6] = (uint8_t)((CAN_RDH0R_DATA6 & CAN1->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA6_Pos);
    aData[7] = (uint8_t)((CAN_RDH0R_DATA7 & CAN1->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA7_Pos);

    /* Release the FIFO */
    if (RxFifo == CAN_RX_FIFO0) /* Rx element is assigned to Rx FIFO 0 */
    {
      /* Release RX FIFO 0 */
      SET_BIT(CAN1->RF0R, CAN_RF0R_RFOM0);
    }
    else /* Rx element is assigned to Rx FIFO 1 */
    {
      /* Release RX FIFO 1 */
      SET_BIT(CAN1->RF1R, CAN_RF1R_RFOM1);
    }



  return 0;// Message read successfully

}


void can_filter_config(uint16_t std_id)
{
	/*Set filter intiailization mode*/
	CAN1->FMR |= CAN_FMR_FINIT;

	/*Set the slave filter to start from 13*/
	CAN1->FMR &=~(CAN_FMR_CAN2SB_Msk);
	CAN1->FMR |=(13 << CAN_FMR_CAN2SB_Pos);

	/*****Filter activation sequence****/
	/*Deactive filter 12*/
	CAN1->FA1R &=~(CAN_FA1R_FACT12);

	/*Set 32-bit scale configuration*/
	CAN1->FS1R  |= CAN_FS1R_FSC12;

	/*Configure filter mode to identifier mask mode*/
	CAN1->FM1R &=~CAN_FM1R_FBM12;

	/*Set identifier*/
	CAN1->sFilterRegister[12].FR1 = (std_id << 21);



	/*Assign filter 12 to FIFO0*/
	CAN1->FFA1R &=~(CAN_FFA1R_FFA12);

	/*Activate filter 12*/
	CAN1->FA1R |= (CAN_FA1R_FACT12);

	/*Clear filter intialization mode*/
	CAN1->FMR &= ~CAN_FMR_FINIT;

}




