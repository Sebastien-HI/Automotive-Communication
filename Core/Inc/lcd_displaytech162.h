/*
 * lcd.h
 *
 *  Created on: 13 sept. 2019
 *      Author: aravey
 */

#ifndef LCD_H_
#define LCD_H_

/* defines */

#define Clear_RS (GPIOB->BSRR |= 0x80000000)
#define Set_RS (GPIOB->BSRR |= 0x00008000)

#define Clear_RW (GPIOB->BSRR |= 0x40000000)
#define Set_RW (GPIOB->BSRR |= 0x00004000)

#define Clear_E (GPIOB->BSRR |= 0x20000000)
#define Set_E (GPIOB->BSRR |= 0x00002000)


/* Function prototypes */

void Init_PortB(void);
void Init_PortDasRead(void);
void Init_PortDasWrite(void);
char lcd_read_byte(void);
char lcd_read_nibble(void);
void lcd_send_nibble(char n);
void lcd_send_byte(char c, char addr);
void lcd_init(void);
void lcd_gotoxy(int x, int y);
void lcd_putc(char c);
void lcd_puts(char line, char *str);
void lcd_clear(void);

#endif /* LCD_H_ */
