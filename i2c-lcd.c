/*
 * i2c-lcd.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Else
 */

#include "i2c-lcd.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

void LCD_Send(uint8_t data, uint8_t mode) {
    uint8_t highNibble = (data & 0xF0) | mode | LCD_BACKLIGHT;
    uint8_t lowNibble = ((data << 4) & 0xF0) | mode | LCD_BACKLIGHT;

    // Send the high nibble
    uint8_t packet = highNibble;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDRESS, &packet, 1, HAL_MAX_DELAY);

    packet = highNibble | ENABLE_BIT;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDRESS, &packet, 1, HAL_MAX_DELAY);

    packet = highNibble & ~ENABLE_BIT;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDRESS, &packet, 1, HAL_MAX_DELAY);

    // Send the low nibble
    packet = lowNibble;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDRESS, &packet, 1, HAL_MAX_DELAY);

    packet = lowNibble | ENABLE_BIT;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDRESS, &packet, 1, HAL_MAX_DELAY);

    packet = lowNibble & ~ENABLE_BIT;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDRESS, &packet, 1, HAL_MAX_DELAY);
}

void LCD_Command(uint8_t cmd){
	LCD_Send(cmd, 0);
}

void LCD_Data(uint8_t data){
	LCD_Send(data, RS_BIT);
}

void LCD_Print(char *str){

	while (*str){
		LCD_Data(*str++);
	}
}

void LCD_Init(void){
	HAL_Delay(50);  // Wait for LCD to power up
    LCD_Send(0x03, 0);  // Initialize in 4-bit mode
    HAL_Delay(5);
    LCD_Send(0x03, 0);
    HAL_Delay(1);
    LCD_Send(0x03, 0);
    LCD_Send(0x02, 0);  // Set to 4-bit mode

	    // Configure LCD
    LCD_Send(0x28, 0);  // Function set: 4-bit, 2-line, 5x8 dots
    LCD_Send(0x08, 0);  // Display off
    LCD_Send(0x01, 0);  // Clear display
    HAL_Delay(2);
    LCD_Send(0x06, 0);  // Entry mode set: Increment cursor
    LCD_Send(0x0C, 0);  // Display on, cursor off
}



