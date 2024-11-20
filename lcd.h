/*
 * lcd.h
 *
 *  Created on: Nov 19, 2024
 *      Author: Else
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "stm32f4xx_hal.h"  // Include HAL library for STM32F4

// Define the LCD control pins and ports
#define RS_Pin GPIO_PIN_0   // Replace with the actual GPIO pin for RS
#define RS_Port GPIOB       // Replace with the actual GPIO port for RS

#define E_Pin GPIO_PIN_1    // Replace with the actual GPIO pin for E
#define E_Port GPIOB        // Replace with the actual GPIO port for E

#define Data_Port GPIOC     // Replace with the actual GPIO port for data pins
#define D4_Pin GPIO_PIN_4   // Replace with the actual GPIO pin for D4
#define D5_Pin GPIO_PIN_5   // Replace with the actual GPIO pin for D5
#define D6_Pin GPIO_PIN_6   // Replace with the actual GPIO pin for D6
#define D7_Pin GPIO_PIN_7   // Replace with the actual GPIO pin for D7

// Function prototypes
void LCD_Init(void);                  // Initialize the LCD
void LCD_Command(uint8_t cmd);        // Send a command to the LCD
void LCD_Data(uint8_t data);          // Send data (character) to the LCD
void LCD_Clear(void);                 // Clear the LCD display
void LCD_SetCursor(uint8_t row, uint8_t col); // Set cursor position
void LCD_Print(char *str);            // Print a string on the LCD


#endif /* INC_LCD_H_ */
