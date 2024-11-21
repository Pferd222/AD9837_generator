/* 
 * File:   lcd1.h
 * Author: Pferd222
 *
 * Created on 2024/08/22, 19:34
 */

#ifndef LCD1_H
#define	LCD1_H

#ifdef	__cplusplus
extern "C" {
#endif

    lcd_write(unsigned char);
    lcd_clear();
    lcd_goto(unsigned char);
    lcd_init();
    void putch(unsigned char);



#ifdef	__cplusplus
}
#endif

#endif	/* LCD1_H */

