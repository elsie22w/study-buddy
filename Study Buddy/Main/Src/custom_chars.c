/*
 * custom_chars.cpp
 *
 *  Created on: Nov 23, 2024
 *      Author: Ashle
 */

#include "custom_chars.h"
#include "i2c-lcd.h"
#include "stm32f4xx_hal.h"

extern void LCD_Command(uint8_t cmd);
extern void LCD_Data(uint8_t data);
extern void LCD_Init(void);


void LCD_CreateCustomChar(uint8_t location, uint8_t charmap[]) {
    // Ensure location is within 0-7
    location &= 0x07;

    // Set CGRAM address (0x40 is the base address)
    LCD_Command(0x40 | (location << 3));

    // Write 8 bytes of custom character data
    for (int i = 0; i < 8; i++) {
        LCD_Data(charmap[i]);
    }
}

void LCD_DisplayCustomChar(uint8_t location) {
    // Write the character index to display it
    LCD_Data(location);
}


