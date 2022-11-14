/*
 * lcd.c
 *
 *  Created on: 13 sept. 2019
 *      Author: aravey
 */


#include "stm32f4xx_hal.h"
#include "lcd_displaytech162.h"

void Init_PortB(void)
{
	  /* Activate Port B and D for LCD Display */
	  SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOBEN);
	  /* Setup GPIO 15,14 and 13 to Output (RS, R/W, E) */
	  GPIOB->MODER |= 0x54000000;
	  /* Output in push-pull mode */
	  GPIOB->OTYPER &= 0xFFFF1FFF;
	  /* Output speed set to High */
	  GPIOB->OSPEEDR |= 0xFC000000;
	  /* No PU/PD resistor */
	  GPIOB->PUPDR &= 0x03FFFFFF;
	  /* Set all to 0 */
	  GPIOB->ODR &= 0xFFFF1FFF;

}

void Init_PortDasRead(void)
{
	  /* Activate Port D for LCD Display */
	  SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIODEN);
	  /* Setup GPIO 11,10,9 and 8 to input (b7,b6,b5,b4) */
	  GPIOD->MODER &= 0xFF00FFFF;
	  /* No PU/PD resistor */
	  GPIOD->PUPDR &= 0xFF00FFFF;

}

void Init_PortDasWrite(void)
{
	  /* Activate Port D for LCD Display */
	  SET_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIODEN);
	  /* Setup GPIO 11,10,9 and 8 to Output (b7,b6,b5,b4) */
	  GPIOD->MODER |= 0x00550000;
	  /* Output in push-pull mode */
	  GPIOD->OTYPER &= 0xFFFFF0FF;
	  /* Output speed set to High */
	  GPIOD->OSPEEDR |= 0x00FF0000;
	  /* No PU/PD resistor */
	  GPIOD->PUPDR &= 0xFF00FFFF;
}

char lcd_read_nibble()
{
	uint32_t data_received;
	Init_PortDasRead();
	Clear_RS;
	Set_RW;
	HAL_Delay(1);
	Set_E;
	HAL_Delay(1);
	data_received=GPIOD->IDR & 0x00000F00 >> 8;
	Clear_E;
	HAL_Delay(1);
	return(data_received);
}

char lcd_read_byte(void)
{
	char data;
	data =  lcd_read_nibble() <<4;
	data =  lcd_read_nibble();
	return(data);
}

void lcd_send_nibble(char n)
{
	Init_PortDasWrite();
	Clear_RW;
	Clear_E;
	HAL_Delay(1);
	GPIOD->ODR = GPIOD->ODR & 0xFFFFF0FF;
	GPIOD->ODR = GPIOD->ODR | (n & 0x0F) <<8;
	HAL_Delay(1);
	Set_E;
	HAL_Delay(2);
	Clear_E;
}
void lcd_send_byte(char c, char addr)
{
	while ( lcd_read_byte() & 0x80 ) ;
	if (addr == 1)
	{
		Set_RS;
	}
	else
	{
		Clear_RS;
	}
	lcd_send_nibble(c>>4);
	lcd_send_nibble(c & 0x0F);

}
void lcd_init()
{
	Clear_RS;
	//Wait 2ms
	HAL_Delay(2);
	for(int i=0;i<3;i++)
	{
		// send dummy nibble
		lcd_send_nibble(0x03);
		// wait 5ms
		HAL_Delay(5);
	}

	// set 4bits mode, 2 lines, 5*8 cells
	lcd_send_byte(0x38,0);
	HAL_Delay(5);
	// Turn off screen, stop blink
	lcd_send_byte(0x08,0);
	HAL_Delay(5);
	// Turn on screen, cursor is blinking
	lcd_send_byte(0x0C,0);
	HAL_Delay(5);
	// Reset to initial position
	lcd_send_byte(0x02,0);
	HAL_Delay(1);
	// Left shift for write
	lcd_send_byte(0x6,0);
	HAL_Delay(1);
	// Reset screen
	lcd_send_byte(0x01,0);
}

void lcd_gotoxy(int x, int y)
{
   int address;

   if(y==2) address=0x40;
   else address=0;

   address+=x-1;
   lcd_send_byte(0x80|address,0);
}

void lcd_putc(char c)
{
	lcd_send_byte(c,1);
}

void lcd_puts(char line, char *str)
{
    uint8_t i = 0;

    if (line == 1) lcd_gotoxy(1, 1);
    else if (line == 2) lcd_gotoxy(1, 2);

    while(*str && (i++ < 16))
    {
	lcd_putc(*str++);
    }
    while(i++ < 16){
    	lcd_putc(' ');
    }
}

void lcd_clear(){
	lcd_send_byte(0,0x01);
}
