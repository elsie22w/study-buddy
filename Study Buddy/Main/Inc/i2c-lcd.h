/*
 * i2c-lcd.h
 *
 *  Created on: Nov 19, 2024
 *      Author: Ashle
 */

#ifndef INC_I2C_LCD_H_
#define INC_I2C_LCD_H_

#include "stm32f4xx_hal.h"

void LCD_Init(void);
void LCD_Send(uint8_t data, uint8_t mode);
void LCD_Command(uint8_t cmd);
void LCD_Clear(void);
void LCD_Cursor_R();
void LCD_Cursor_L();
void LCD_Data(uint8_t data);
void LCD_Print(char *str);

void LCD_Main_Menu();

#define LCD_I2C_ADDRESS (0x27 << 1)
#define LCD_BACKLIGHT 0x08
#define LCD_NO_BACKLIGHT 0x00
#define ENABLE_BIT 0x04
#define RS_BIT 0x01


#endif /* INC_I2C_LCD_H_ */
