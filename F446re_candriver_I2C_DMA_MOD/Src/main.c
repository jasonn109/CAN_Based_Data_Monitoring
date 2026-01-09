#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f446xx.h"
#include "rcc.h"
#include "candriver.h"
#include "uart.h"
#include "i2c_dma.h"
#include "fonts.h"
#include "ssd1306.h"




uint8_t rx_data[6];
uint8_t tx_data[5];
int16_t x,y,z;
float xg, yg,zg;
uint32_t tx_mailbox[3];

can_rx_header_typedef rx_header;
can_tx_header_typedef tx_header;

uint8_t count = 0;

void CAN1_RX0_IRQHandler(void)
{
	if((CAN1->RF0R & CAN_RF0R_FMP0) != 0U)
	{
		can_get_rx_message(CAN_RX_FIFO0, &rx_header, rx_data);
		count++;


		printf("FIFIO 0 rx_data 0 %d\n\r",rx_data[0]);
		printf("FIFIO 0 rx_data 1 %d\n\r",rx_data[1]);
		printf("FIFIO 0 rx_data 2 %d\n\r",rx_data[2]);
		printf("FIFIO 0 rx_data 3 %d\n\r",rx_data[3]);
		printf("FIFIO 0 rx_data 4 %d\n\r",rx_data[4]);
		printf("FIFIO 0 rx_data 5 %d\n\r",rx_data[5]);

	}

}

void CAN1_RX1_IRQHandler(void)
{
	if((CAN1->RF1R & CAN_RF1R_FMP1) != 0U)
	{
		can_get_rx_message(CAN_RX_FIFO1, &rx_header, rx_data);
		count++;


		printf("FIFIO 1 rx_data 0 %d\n\r",rx_data[0]);
		printf("FIFIO 1 rx_data 1 %d\n\r",rx_data[1]);
		printf("FIFIO 1 rx_data 2 %d\n\r",rx_data[2]);
		printf("FIFIO 1 rx_data 3 %d\n\r",rx_data[3]);
		printf("FIFIO 1 rx_data 4 %d\n\r",rx_data[4]);
		printf("FIFIO 1 rx_data 5 %d\n\r",rx_data[5]);
	}

}
int main()
{
	char buffltx[10],bufflty[10],buffltz[10];
	can_gpio_init();
	can_params_init(CAN_MODE_NORMAL);
	can_filter_config(0x244);
	can_start();
	debug_uart_init();


	I2C1_Init();
	dma1_stream6_i2c1_tx_init();
	SSD1306_Init (); // initialise the display


    printf("Receiver ready\n\r");
	while(1)
	{

		x=((rx_data[1]<<8)|rx_data[0]);
		y=((rx_data[3]<<8)|rx_data[2]);
		z=((rx_data[5]<<8)|rx_data[4]);


//		xg = (x * 0.0078);
//		yg = (y * 0.0078);
//		zg = (z * 0.0078);

		sprintf (buffltx, "X: %d", x);
		sprintf (bufflty, "Y: %d", y);
		sprintf (buffltz, "Z: %d", z);
		SSD1306_GotoXY (0,0); // goto 10, 10
		SSD1306_Puts (buffltx, &Font_7x10, 1); // print Hello
		SSD1306_GotoXY (0,17 );
		SSD1306_Puts (bufflty, &Font_7x10, 1);
		SSD1306_GotoXY (0,34 );
		SSD1306_Puts (buffltz, &Font_7x10, 1);
		SSD1306_UpdateScreen(); // update screen

//		for(int i=0; i<1000; i++)
//		{
//			for(int j=0; j<1000; j++);
//		}
		//SSD1306_Clear();

	}
}
