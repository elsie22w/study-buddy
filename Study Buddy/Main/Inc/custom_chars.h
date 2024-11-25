/*
 * custom_chars.h
 *
 *  Created on: Nov 23, 2024
 *      Author: Ashle
 */

#ifndef INC_CUSTOM_CHARS_H_
#define INC_CUSTOM_CHARS_H_


#include <stdint.h>

// Define custom characters

void LCD_DisplayCustomChar(uint8_t location);
void LCD_CreateCustomChar(uint8_t location, uint8_t charmap[]);


#endif /* INC_CUSTOM_CHARS_H_ */
