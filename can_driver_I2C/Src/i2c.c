/*
 * i2c.c
 *
 *  Created on: Oct 15, 2025
 *      Author: vivian H C
 */
#include "stm32f1xx.h"


#define GPIOBEN              (1U<<3)
#define I2C1EN               (1U<<21)
#define I2C_CR1_SWRST        (1U<<16)
#define I2C_CR1_PE           (1U<<0)
#define I2C_CR2_FREQ_Pos       (0U)

#define SR2_BUSY             (1U<<1)
#define CR1_START            (1U<<8)
#define SR1_SB               (1U<<0)
#define SR1_ADDR            (1U<<1)
#define SR1_TXE              (1U<<7)
#define CR1_ACK              (1U<<10)
#define CR1_STOP             (1U<<9)
#define SR1_RXNE             (1U<<6)
#define SR1_BTF              (1U<<2)

#define AFEN				(1U<<0)


/*Pinout
 * PB7 -----SDA
 * PB6 -----SCL
 */



void I2C1_init(void)
{
	/*Enable Clock access to GPIOB*/
	RCC->APB2ENR |= AFEN;
	RCC->APB2ENR |= GPIOBEN;

	/*Enable Clock access to I2C1*/
	RCC->APB1ENR |= I2C1EN;

	/*Set PB7 and PB6 mode to alternate function*/
	/*Set PB7 and PB6 output type to open drain*/
	GPIOB->CRL &= ~(0xFF000000);
	GPIOB->CRL |= (0xFF<<24);

    /*Temporarily disable the peripheral to apply settings*/
    I2C1->CR1 &= ~I2C_CR1_PE;

    /*Reset the I2C peripheral to clear any previous state*/
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    /*Set the peripheral clock frequency in CR2 register (in MHz)*/
    /*This value must match your PCLK1(APB1 36MHZ) frequency*/
    I2C1->CR2 |= (16 << I2C_CR2_FREQ_Pos);   //36MHz

    /*Configure I2C clock for 100 kHz (Standard Mode)
      Formula: CCR = PCLK1 / (2 * I2C_FREQ)
      For PCLK1 = 36MHz and I2C_FREQ = 100kHz, CCR = 36,000,000 / 200,000 = 180*/
    I2C1->CCR = 80;

    /* Configure the Rise Time (TRISE) register
       Formula: TRISE = (Maximum SCL rise time / PCLK1 period) + 1
       For 100kHz, max rise time is 1000ns.
       TRISE = (1000ns / (1/36MHz)) + 1 = 36 + 1 = 37*/
    I2C1->TRISE = 17;

    /* Enable the peripheral by setting the PE bit*/
    I2C1->CR1 |= I2C_CR1_PE;

}

void I2C1_byteRead(char saddr,char maddr,char* data)
{
	volatile int tmp;

	/*wait until bus not busy*/
 	while(I2C1->SR2 & (SR2_BUSY)){}

	/*Generate start*/
	I2C1->CR1 |= CR1_START;

	/*wait until start flag is set*/
	while(!(I2C1->SR1 & (SR1_SB))){}

	/*Transmit slave address + write*/
	I2C1->DR = saddr<<1;

	/*wait until addr flag is set*/
	while(!(I2C1->SR1 & (SR1_ADDR))){}

	/*Clear addr flag*/
	tmp = I2C1->SR2;

	/*send memory address*/
	I2C1->DR = maddr;

	/*wait untill transmitter empty*/
	while (!(I2C1->SR1 & SR1_TXE)) {}

	/*Generate restart*/
	I2C1->CR1 |= CR1_START;

	/*wait until start flag is set*/
	while(!(I2C1->SR1 & SR1_SB)){}

	/*Transmit slave address + read*/
	I2C1->DR = saddr<<1|1;

	/*wait until addr flag is set*/
	while(!(I2C1->SR1 & (SR1_ADDR))){}

	/*Disable ACK*/
	I2C1->CR1 &= ~CR1_ACK;

	/*Clear addr flag*/
	tmp = I2C1->SR2;

	/* Generate stop after data received*/
	I2C1->CR1 |= CR1_STOP;

	/*wait until RXNE flag is set*/
	while(!(I2C1->SR1 & SR1_RXNE)){}

    /*Read data from DR*/
	*data++ = I2C1->DR;
}


void I2C1_burstRead(char saddr,char maddr,int n,char* data){
	volatile int tmp;

	/*wait until bus not busy*/
	while(I2C1->SR2 & (SR2_BUSY)){}

	/*Generate start*/
	I2C1->CR1 |= CR1_START;

	/*wait until start flag is set*/
	while(!(I2C1->SR1 & SR1_SB)){}


	/*Transmit slave address + write*/
	I2C1->DR = saddr<<1;

	/*wait until addr flag is set*/
	while(!(I2C1->SR1 & (SR1_ADDR))){}

	/*Clear addr flag*/
	tmp = I2C1->SR2;

	/*wait until transmitter empty*/
	while (!(I2C1->SR1 & SR1_TXE)) {}

	/*send memory address*/
	I2C1->DR = maddr;

	/*wait until transmitter empty*/
	while (!(I2C1->SR1 & SR1_TXE)) {}

	/*Generate restart*/
	I2C1->CR1 |= CR1_START;

	/*wait until start flag is set*/
	while(!(I2C1->SR1 & SR1_SB)){}

	/*Transmit slave address + read*/
	I2C1->DR = saddr<<1|1;

	/*wait until addr flag is set*/
	while(!(I2C1->SR1 & (SR1_ADDR))){}

	/*Clear addr flag*/
	tmp = I2C1->SR2;

	/*Enable ACK*/
	I2C1->CR1 |= CR1_ACK;

	while(n>0U)
	{
		/*if one byte*/
		if(n==1u)
		{
			/*Disable ACK*/
			I2C1->CR1 &= ~CR1_ACK;

			/* Generate stop after data received*/
			I2C1->CR1 |= CR1_STOP;

			/*wait until RXNE flag is set*/
			while(!(I2C1->SR1 & SR1_RXNE)){}

		    /*Read data from DR*/
			*data++ = I2C1->DR;
			break;
		}
		else
		{
			/*wait until RXNE flag is set*/
			while(!(I2C1->SR1 & SR1_RXNE)){}

			/*Read the data from DR*/
			(*data++) = I2C1->DR;

			n--;
		}
	}


}


void I2C1_burstWrite(char saddr,char maddr,int n,char* data){
	volatile int tmp;

		/*wait until bus not busy*/
		while(I2C1->SR2 & (SR2_BUSY)){}

		/*Generate start*/
		I2C1->CR1 |= CR1_START;

		/*wait until start flag is set*/
		while(!(I2C1->SR1 & SR1_SB)){}


		/*Transmit slave address + write*/
		I2C1->DR = saddr<<1;

		/*wait until addr flag is set*/
		while(!(I2C1->SR1 & (SR1_ADDR))){}

		/*Clear addr flag*/
		tmp = I2C1->SR2;

		/*wait until transmitter empty*/
		while (!(I2C1->SR1 & SR1_TXE)) {}

		/*send memory address*/
		I2C1->DR = maddr;

		for (int i=0;i<n;i++)
		{
			/*wait untill data reg empty*/
			while (!(I2C1->SR1 & (SR1_TXE))){}

			/* Transmit memory address*/
			I2C1->DR = *data++;
		}

		/*wait until transfer finished */
		while(!(I2C1->SR1 & (SR1_BTF))){}

		/* Generate Stop */
		I2C1->CR1 |= CR1_STOP;

}








