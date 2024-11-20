/*
 * i2c-lcd.h
 *
 *  Created on: Nov 19, 2024
 *      Author: Ashle
 */

#ifndef INC_I2C_LCD_H_
#define INC_I2C_LCD_H_

#include "stm32f4xx_hal.h"

void lcd_init(void);
void lcd_command(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_string(char *str);

#define LCD_I2C_ADDRESS (0x27 << 1)
#define LCD_BACKLIGHT 0x08
#define LCD_NO_BACKLIGHT 0x00
#define ENABLE_BIT 0x04
#define RS_BIT 0x01


#endif /* INC_I2C_LCD_H_ */
