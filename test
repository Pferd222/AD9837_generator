/*
 * File:   newmain.c
 * Author: Pferd222
 *
 * Created on 2024/08/01, 10:01
 */


#include <xc.h>
#include <math.h>
#include <stdio.h>
#include "lcd1.h"
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = OFF       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = ON    // Clock Out Enable (CLKOUT function is enabled on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable (All VCAP pin functionality is disabled)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

#define _XTAL_FREQ 16000000
#define _FREQ_REG_MAX 268435456
//RA0:SCLK RA1:ESYNC RA2:SDATA 

__EEPROM_DATA (0b11101000, 0b00000011, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);

void send(int);
void clk(void);
void SetFrequency(long, int);
void setup(void);
void read(void);
void memory(void);
void memorysave(void);
void memorysave1(void);
void interrupt isr(void);
void maxfreq(void);
void wave(void);
void lcd(long, int);
const long refFreq = 4000000;
const int SINE = 0b0000000000000000; //0x2000
const int TRIANGLE = 0b0000000000000010; //0x2002
const int SQUARE = 0b0000000000100000; //0x2020
//const int SQUARE_half = 0b0010000000101000;

long OUTfrequency;

long tmp3;

int count;
long maxfreqout;
int waveform;
int freqbefore;
int wavebefore;

void main(void) {
    
    setup();
    

    while(1){
        
        memory();
        
        maxfreq();
        
        wave();

    }
        
    return;
}

void interrupt isr(){
    
            
    if ((RB0 == 1)&&(RB2 ==0)&&(RB4 == 0)){
        
 
        if (RB1 == 1){
            count -= 1;
        }else{
            count += 1;
        }

        if (count >= 2){
            OUTfrequency += maxfreqout;
            count = 0;
            lcd(OUTfrequency, waveform);
        }
        else{
            if (count <= -2){
                long gap = OUTfrequency - maxfreqout;
                lcd(OUTfrequency, waveform);
                if (!(gap <= 0)){
                    OUTfrequency -= maxfreqout;
                    lcd(OUTfrequency, waveform);
                }else{}
            
                count = 0;
            }else{
            }
        }
        SetFrequency(OUTfrequency, waveform);  
        
        

        
    }else if (RB2 == 1){
        
        memorysave();
        
        
    }else if (RB4 == 1){
    
        memorysave1();
    
    }else{}

    IOCBF = 0;//frag off
}

void wave(){
    wavebefore;
    int tmp = PORTD;
    int wave1 = tmp & 0b01110000;
    wave1 = wave1 >> 4;
    
    switch(wave1){
        case 0b001:
            waveform = SINE;
            break;
        case 0b010:
            waveform = TRIANGLE;
            break;
        case 0b100:
            waveform = SQUARE;
            break;
    }
    
    if(!(wavebefore == waveform)){
        SetFrequency(OUTfrequency, waveform);
        
        lcd(OUTfrequency, waveform);
        
        wavebefore = waveform;
        }else{}

}
void lcd(long number, int form){
    
    lcd_init();  //LCD初期化
    
    
    if (number >= 1000){
        number = number / 1000;
        lcd_goto(0);
        printf("%ld", number);
        lcd_goto(3);
        printf("kHz");
    
    }else{
        lcd_goto(0);
        printf("%ld", number);
        lcd_goto(4);
        printf("Hz");
    
    }
    
    switch(form){
        case 0:
            lcd_goto(8);
            printf("SINE");
            break;
        case 0b10:
            lcd_goto(8);
            printf("TRIANGLE");
            break;
        case 0b100000:
            lcd_goto(8);
            printf("SQUARE");
            break;
    }
    
} 


void maxfreq(){
    

    int tmp = PORTD;
    int freqrotary = tmp & 0b00001111; 

    
    switch(freqrotary){
        case 0b0001:
            maxfreqout = 10;
            
            break;
        case 0b0010:
            maxfreqout = 100;

            break;
        case 0b0100:
            maxfreqout = 1000;
            
            break;
        case 0b1000:
            maxfreqout = 10000;
            break;

    }

        if(!(freqbefore == maxfreqout)){
            OUTfrequency = maxfreqout;
            SetFrequency(OUTfrequency, waveform);
            
            lcd(OUTfrequency, waveform);
            
            freqbefore = maxfreqout;

        }else{}
}


void memory(){
    if(RE0){
        long read0 = eeprom_read(0);//下位ビット
        long read1 = eeprom_read(1);//上位ビット
        long read2 = eeprom_read(2);//上位ビット
        long read3 = eeprom_read(3);
        read1 = read1 << 8;
        read2 = read2 << 16;
        long readfrequency = read1 + read0 + read2;
        
        SetFrequency(readfrequency, read3);
        
        lcd(readfrequency, read3);
        lcd_goto(40);
        printf("*Memory1 read*");
        
        while(RE0);
        
        SetFrequency(OUTfrequency, waveform);
        
        lcd(OUTfrequency, waveform);
        
    }else if(RE1){
        
        long read4 = eeprom_read(4);//下位ビット
        long read5 = eeprom_read(5);//上位ビット
        long read6 = eeprom_read(6);//上位ビット
        long read7 = eeprom_read(7);
        read5 = read5 << 8;
        read6 = read6 << 16;
        long readfrequency = read4 + read5 + read6;
        
        SetFrequency(readfrequency, read7);
        
        lcd(readfrequency, read7);
        lcd_goto(40);
        printf("*Memory2 read*");
        
        while(RE1);
        
        SetFrequency(OUTfrequency, waveform);
        
        lcd(OUTfrequency, waveform);
    
    }    
    
    
}

void memorysave(){
    
        int byte0 = OUTfrequency & 0b11111111;
        int savefrequency = OUTfrequency >> 8;
        int byte1 = savefrequency & 0b11111111;
        savefrequency = savefrequency >> 8;
        int byte2 = savefrequency & 0b11111111;
        int byte3 = waveform;
        eeprom_write(0, byte0);
        eeprom_write(1, byte1);
        eeprom_write(2, byte2);
        eeprom_write(3, byte3);
        
        lcd(OUTfrequency, waveform);
        lcd_goto(40);
        printf("*Memory1 saving*");
        __delay_ms(1000);
        
        lcd(OUTfrequency, waveform);
        
}

void memorysave1(){
    
        int byte4 = OUTfrequency & 0b11111111;
        int savefrequency = OUTfrequency >> 8;
        int byte5 = savefrequency & 0b11111111;
        savefrequency = savefrequency >> 8;
        int byte6 = savefrequency & 0b11111111;
        int byte7 = waveform;
        eeprom_write(4, byte4);
        eeprom_write(5, byte5);
        eeprom_write(6, byte6);
        eeprom_write(7, byte7);
        
        lcd(OUTfrequency, waveform);
        lcd_goto(40);
        printf("*Memory2 saving*");
        __delay_ms(1000);
        
        lcd(OUTfrequency, waveform);
        
}

void SetFrequency(long frequency, int Waveform) {
  long FreqWord = frequency * (_FREQ_REG_MAX / refFreq);
 
  int MSB = (FreqWord & 0xFFFC000) >> 14;
  int LSB = FreqWord & 0x3FFF;
 
  LSB |= 0b0100000000000000; //=0x4000
  MSB |= 0b0100000000000000; //=0x4000
 

  send(0b0010000000000000); //制御ワード書き込み + reset on
  
  send(LSB);
  
  send(MSB);
 
  //long isou = 0b1100000000000000;
  //send(isou); //位相シフトはゼロ
  
  send(Waveform);
}



void send(int data){
    int i;
    RA1 = 0; //ESYNCを0にセットしておく。
    __delay_us(1);
    for (i = 0; i < 16; i++){
        int tmp1 = data & 0b1000000000000000;
        if (tmp1){
            RA2 = 1;
        }else{
            RA2 = 0;
        }
        __delay_us(1);
        clk();
        __delay_us(1);
        data = data << 1;
        __delay_us(1);
    }
    RA1 = 1; //ESYNCを1にセットしておく。
    RA0 = 1; //SCLKを1にセットしておく。
    __delay_us(1);
}

void clk(){
    RA0 = 0;
    __delay_us(1);
    RA0 = 1;
}

void setup(){
             
    OSCCON = 0b01111010;
    // a/d
    ANSELA = 0b00000000;
    ANSELB = 0b00000000;
    
    ANSELD = 0b00000000;
    ANSELE = 0b00000000;
    
    // i/o
    TRISA = 0b00000000;
    TRISB = 0b00010111;
    TRISC = 0b00000000;
    TRISD = 0b01111111;
    TRISE = 0b00000011;
    // LCD初期化
    PORTC = 0;
    lcd_init();  //LCD初期化
    // spi
    RA1 = 1; //ESYNCを1にセットしておく。
    RA0 = 1; //SCLKを1にセットしておく。
    
    //ADCON0 = 0b00001101; //CHS = AN3, GO = 0, ADON(有効化) = 1
    //ADCON1 = 0b10000000; //ADFM = 1 (右詰め), ADCS = 000

    //　enable rising edge when interrupt
    IOCBP = 0b00010101;//positive
    IOCBN = 0b00000000;//negative
    
    IOCIE = 1;//割り込みを許可
    GIE = 1;//割り込みの元栓
    
    
    wavebefore = SINE;
    freqbefore = 0;
    count = 0;
}
