/*
 * i2c_dma.c
 *
 *  Created on: Oct 18, 2025
 *      Author: JASON PEREIRA
 */
#include "i2c_dma.h"

/*
PB8 : I2C1 SCL
PB9 : I2C1 SDA
*/
#include "stm32f4xx.h"
#include "stm32f446xx.h"
#include "stddef.h"

#define GPIOBEN				(1U<<1)
#define I2C1EN				(1U<<21)
#define CR1_SWRST			(1U<<15)
#define CR1_NOSTRETCH  		(1U<<7)
#define CR1_ENGC			(1U<<6)
#define CR2_DMAEN			(1U<<11)
#define CR2_LAST			(1U<<12)
#define CR1_PE				(1U<<0)
#define DMA1EN				(1U<<21)
#define DMA_SCR_MINC		(1U<<10)
#define DMA_SCR_TCIE		(1U<<4)
#define DMA_SCR_EN  		(1U<<0)

#define HIFCR_CTCIF6		(1U<<21)
#define HIFCR_CTCIF5		(1U<<11)


#define HIFSR_TCIF5		(1U<<11)
#define HIFSR_TCIF6		(1U<<21)

#define PERIPH_CLK			16

#define SR2_BUSY			(1U<<1)

#define I2C_100KHZ				80   // 0B 0101 0000
#define SD_MODE_MAX_RISE_TIME	17

#define CR1_START			(1U<<8)
#define SR1_SB				(1U<<0)
#define SR1_ADDR			(1U<<1)
#define SR1_TXE				(1U<<7)
#define CR1_ACK				(1U<<10)
#define SR1_BTF				(1U<<2)
#define CR1_STOP			(1U<<9)


volatile uint8_t g_tx_cmplt;

volatile uint8_t i2c1_dma = 0;
//
void I2C1_Init(void) {
    // Enable clocks for I2C1
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;  // Enable I2C1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable GPIOB clock

    GPIOB->MODER &= ~GPIO_MODER_MODER8;  // Clear mode bits
    GPIOB->MODER |= 2<<GPIO_MODER_MODER8_Pos; // Set alternate function mode
    GPIOB->MODER &= ~GPIO_MODER_MODER9; // Clear mode bits
    GPIOB->MODER |= 2<<GPIO_MODER_MODER9_Pos; // Set alternate function mode

    GPIOB->OTYPER |= (1 << 8) | (1 << 9); // Set open-drain configuration

    GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED8; // Clear mode bits
    GPIOB->OSPEEDR |= 3UL<<GPIO_OSPEEDR_OSPEED8_Pos;
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED9; // Clear mode bits
    GPIOB->OSPEEDR |= 3UL<<GPIO_OSPEEDR_OSPEED9_Pos;

    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD8;
    GPIOB->PUPDR |= 1UL<<GPIO_PUPDR_PUPD8_Pos;
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD9;
    GPIOB->PUPDR |= 1UL<<GPIO_PUPDR_PUPD9_Pos;

    GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL8;
    GPIOB->AFR[1] |= 4UL<<GPIO_AFRH_AFSEL8_Pos;
    GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL9;
    GPIOB->AFR[1] |= 4UL<<GPIO_AFRH_AFSEL9_Pos;

    //reset the I2C module
    I2C1->CR1|= I2C_CR1_SWRST;

    //disable reset
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    // I2C1 configuration
    I2C1->CR1 &= ~I2C_CR1_PE;             // Disable I2C before configuration

    /*Enable clock stretching*/
    I2C1->CR1 &=~CR1_NOSTRETCH;

    /*Disable General Call*/
    I2C1->CR1 &=~CR1_ENGC;

    /*Select to use DMA*/
    I2C1->CR2 |=CR2_DMAEN;

    /*Enable LAST*/
    I2C1->CR2 |=CR2_LAST;
    //I2C1->CCR &= ~I2C_CCR_FS;
    //I2C1->CCR |= I2C_CCR_FS;
    //I2C1->CCR &= ~I2C_CCR_DUTY;

    I2C1->CR2 = (16 & I2C_CR2_FREQ_Msk);  // Set APB1 clock frequency (16 MHz)
    //I2C1->CCR &= ~I2C_CCR_CCR;
    I2C1->CCR = 80;                      // Set clock control register for 100kHz in Standard mode
    //I2C2->CCR |= 38<<I2C_CCR_CCR_Pos;
    I2C1->TRISE = 17;                     // Set maximum rise time
    I2C1->CR1 |= I2C_CR1_PE;              // Enable I2C peripheral
}

void i2c1_init(void)
{
	/********I2C GPIO Configuration*************/
	/*Enable clock access to GPIOB*/
	RCC->AHB1ENR |= GPIOBEN;

	/*Set PB8 and PB9 mode to alternate function mode*/
	/*PB8*/
	GPIOB->MODER &= ~(1U<<16);
	GPIOB->MODER |= (1U<<17);


	/*PB9*/
	GPIOB->MODER &= ~(1U<<18);
	GPIOB->MODER |= (1U<<19);


	/*Set PB8 and PB9 alternate function type to I2C1 (AF4)  */
	/*PB8*/
	GPIOB->AFR[1] &=~(1U<<0);
	GPIOB->AFR[1] &=~(1U<<1);
	GPIOB->AFR[1] |=(1U<<2);
	GPIOB->AFR[1] &=~(1U<<3);

	/*PB9*/
	GPIOB->AFR[1] &=~(1U<<4);
	GPIOB->AFR[1] &=~(1U<<5);
	GPIOB->AFR[1] |=(1U<<6);
	GPIOB->AFR[1] &=~(1U<<7);

    GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED8; // Clear mode bits
    GPIOB->OSPEEDR |= 3UL<<GPIO_OSPEEDR_OSPEED8_Pos;
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED9; // Clear mode bits
    GPIOB->OSPEEDR |= 3UL<<GPIO_OSPEEDR_OSPEED9_Pos;

    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD8;
    GPIOB->PUPDR |= 1UL<<GPIO_PUPDR_PUPD8_Pos;
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD9;
    GPIOB->PUPDR |= 1UL<<GPIO_PUPDR_PUPD9_Pos;

	/*SCL and SDA respectively*/
	/*Set output type of PB8 and PB9 to open-drain*/
	GPIOB->OTYPER |=(1U<<8);
	GPIOB->OTYPER |=(1U<<9);

	/********I2C  Configuration*************/
	/*Enable clock access to I2C1*/
	RCC->APB1ENR |= I2C1EN;

	/*Reset I2C module*/
	I2C1->CR1 = CR1_SWRST;


	/*Release the reset*/
	I2C1->CR1 &= ~CR1_SWRST;

	/*Enable clock stretching*/
	I2C1->CR1 &=~CR1_NOSTRETCH;

	/*Disable General Call*/
	I2C1->CR1 &=~CR1_ENGC;

	/*Select to use DMA*/
	I2C1->CR2 |=CR2_DMAEN;

	/*Enable LAST*/
	//I2C1->CR2 |=CR2_LAST;

	/*Set source clock speed*/
	I2C1->CR2 |=PERIPH_CLK;

	 /*Set I2C to standard mode, 100kHz clock*/
	I2C1->CCR = I2C_100KHZ; /*Based on Computation*/

	I2C1->TRISE = SD_MODE_MAX_RISE_TIME;

	/*Enable I2C module*/
	I2C1->CR1 |=CR1_PE;

}
// DMA 1 stream 6 channel 1 TX


void dma1_stream6_i2c1_tx_init(void)
{
	/*Enable clock access DMA*/
	RCC->AHB1ENR |=DMA1EN;

	/*Disable DMA stream*/
	DMA1_Stream6->CR = 0;

	/*Wait till DMA Stream is disabled*/
	while((DMA1_Stream6->CR & DMA_SCR_EN)){}


	/*Select DMA channel : CH1*/
	DMA1_Stream6->CR |=(1U<<25);
	DMA1_Stream6->CR &=~(1U<<26);
	DMA1_Stream6->CR &=~(1U<<27);

	/*Enable Mem Addr increment*/
	DMA1_Stream6->CR |=DMA_SCR_MINC;

	/*Enable Transfer Complete Interrupt*/
	DMA1_Stream6->CR |=DMA_SCR_TCIE;

	/*Set Transfer direction :  Mem to Periph*/
	DMA1_Stream6->CR |=(1U<<6);
	DMA1_Stream6->CR &=~(1U<<7);

	/*Enable Stream Interrupt in NVIC*/
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}


void dma1_stream6_i2c1_transfer(uint8_t *msg_to_send, uint32_t msg_len)
{
	if( NULL != msg_to_send )
	{
		 g_tx_cmplt = 0;
		 //i2c1_dma = 1;
		/*Clear interrupt flags*/
		//DMA1->HIFCR = HIFCR_CTCIF6;
		DMA1->HIFCR |= HIFCR_CTCIF6;

		/*Set Peripheral address*/
		DMA1_Stream6->PAR =	(uint32_t)(&(I2C1->DR));

		/*Set Memory address*/
		DMA1_Stream6->M0AR = (uint32_t)msg_to_send;

		/*Set transfer length*/
		DMA1_Stream6->NDTR = msg_len;

		/*Enable the DMA Stream*/
		DMA1_Stream6->CR |= DMA_SCR_EN;

	}
	else
	{
		//Do something...
	}
}


void i2c_dma_write(uint8_t slave_addr, uint8_t *p_write_buff, uint16_t num_of_bytes)
{

	//while(i2c1_dma);
	/*Wait while BUSY flag is set*/
	while((I2C1->SR2 & SR2_BUSY)){}

	/*Generate START condition*/
	I2C1->CR1 |= CR1_START;

	/*Wait until the SB flag  is set*/
	while(!(I2C1->SR1 & SR1_SB)){}

	/*Read SR1*/
	 I2C1->SR1;

	/*Send Slave Addr "Write"*/
	 I2C1->DR = (slave_addr<<1|0);

	/*Wait for Addr Flag to be set*/
	 while(!((I2C1->SR1)& SR1_ADDR)){}

	 (void)I2C1->SR1; (void)I2C1->SR2;

	 /* Wait until data register empty */
	  while (!(I2C1->SR1 & (I2C_SR1_TXE))){}

	/*Call DMA transfer function*/
	 dma1_stream6_i2c1_transfer(p_write_buff,num_of_bytes);


	/*Read SR1*/
	 I2C1->SR1;

	/*Read SR2*/
	 I2C1->SR2;

	 while (!g_tx_cmplt);

}

void I2C1_burstWrite(uint8_t saddr, uint8_t maddr, uint16_t n, uint8_t* data) {

//	volatile int tmp;

	//while (i2c1_dma);
	// Disable DMA requests from I2C1
	I2C1->CR2 &= ~CR2_DMAEN;

	 /* Wait until bus not busy */
	 while (I2C1->SR2 & (I2C_SR2_BUSY)){}

     /* Generate start */
    I2C1->CR1 |= I2C_CR1_START;

    /* Wait until start flag is set */
    while (!(I2C1->SR1 & (I2C_SR1_SB))){}

    /* Transmit slave address + write*/
    I2C1->DR = saddr << 1;

    /* Wait until addr flag is set */
    while (!(I2C1->SR1 & (I2C_SR1_ADDR))){}

    /* Clear addr flag */
    I2C1->SR2;

    /* Wait until data register empty */
    while (!(I2C1->SR1 & (I2C_SR1_TXE))){}

    /* Send memory address */
    I2C1->DR = maddr;

    while (!(I2C1->SR1 & I2C_SR1_BTF));

    for (uint16_t i = 0; i < n; i++) {

     /* Wait until data register empty */
        while (!(I2C1->SR1 & (I2C_SR1_TXE))){}

      /* Transmit memory address */
      I2C1->DR = *data++;
    }

    /* Wait until transfer finished */
    while (!(I2C1->SR1 & (I2C_SR1_BTF))){}

    /* Generate stop */
   I2C1->CR1 |= I2C_CR1_STOP;

   // Re-enable DMA afterwards if needed
  I2C1->CR2 |= CR2_DMAEN;
}

void DMA1_Stream6_IRQHandler(void)
{

	if((DMA1->HISR) & HIFSR_TCIF6)
	{
		//do_ssomething

		i2c1_dma = 0;
		/*Clear the flag*/
			DMA1->HIFCR |= HIFCR_CTCIF6;

		/*Generate STOP condition*/
		 I2C1->CR1 |= CR1_STOP;

		 g_tx_cmplt = 1;
	}
}
