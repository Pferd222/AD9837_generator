/*
 * File:   lcd_16MHz.c
 * Author: Pferd222
 *
 * Created on 2024/08/22, 19:58
 */


#include <xc.h>
#include "lcd1.h"

#define _XTAL_FREQ 16000000

#define LCD_RS RC5
#define LCD_EN RC4

#define LCD_DATA PORTC

#define LCD_STROBE() ((LCD_EN=1),(LCD_EN=0))

lcd_write(unsigned char c)
{
    
    __delay_us(80);
    if(LCD_RS) LCD_DATA=(((c>>4)&0x0F)|0x20);
    else LCD_DATA=((c>>4)&0x0F);
    LCD_STROBE();
    if(LCD_RS) LCD_DATA=((c&0x0F)|0x20);
    else LCD_DATA=(c&0x0F);
    LCD_STROBE();
    
}

lcd_clear()
{
    LCD_RS = 0;
    lcd_write(0x1);
    __delay_ms(4);
    
}

lcd_goto(unsigned char pos)
{
    
    LCD_RS = 0;
    lcd_write(0x80+pos);
    
}

lcd_init()
{
    LCD_RS = 0;
    LCD_EN = 0;
    
    __delay_ms(30);
    	LCD_DATA=0x03;
	LCD_STROBE();
	__delay_ms(10);
	LCD_STROBE();
	__delay_us(400);
	LCD_STROBE();
	__delay_us(400);
	LCD_DATA = 0x02;
	LCD_STROBE();

	lcd_write(0x28);
	lcd_write(0x0C);
	lcd_clear();
	lcd_write(0x06);
}

void putch(unsigned char c)				/* �f�[�^��1�o�C�g�������� */
{
	LCD_RS = 1;
	lcd_write( c );
}
