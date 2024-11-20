/*
 * lcd.c
 *
 *  Created on: Nov 19, 2024
 *      Author: Elsie
 */

#include "lcd.h"
#include "main.h"

//// define required pins
//#define RS_Pin GPIO_PIN_0
//#define RS_Port GPIOA
//#define E_Pin GPIO_PIN_1
//#define E_Port GPIOA
//#define Data_Port GPIOB

// helper functions
void LCD_Command(uint8_t cmd){ // CHANGE ?
	// Function to send a command to LCD
	HAL_GPIO_WritePin(RS_Port, RS_Pin, GPIO_PIN_RESET); //set RS to 0 for command
	HAL_GPIO_WritePin(E_Port, E_Pin, GPIO_PIN_SET); // enable pin is toggled to allow data to be read
	HAL_GPIO_WritePin(Data_Port, cmd >> 4); //send higher nibble (most significant 4-bits)
	HAL_GPIO_WritePin(E_Port, E_Pin, GPIO_PIN_RESET);

	HAL_Delay(1);

	HAL_GPIO_WritePin(E_Port, E_Pin, GPIO_PIN_SET);
	HAL_CPIO_WritePin(Data_Port, cmd & 0x0F); // send lower nibble (least significant 4-bits)
	HAL_GPIO_WritePin(E_Port, E_Pin, GPIO_PIN_RESET);

	HAL_Delay(2); // command execution time ?
}

void LCD_Data(uint8_t data){ // ? combine these functions (?) helper of helper
	// function to send data to LCD
	HAL_GPIO_WritePin(RS_Port, RS_Pin, GPIO_PIN_SET); // set RS to 1 for data
	HAL_GPIO_WritePin(E_Port, E_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Data_Port, data >> 4); // send higher nibble
	HAL_GPIO_WritePin(E_Port, E_Pin, GPIO_PIN_RESET);

	HAL_Delay(1);

	HAL_GPIO_WritePin(E_Port, E_Pin, GPIO_PIN_SET);
	HAL_CPIO_WritePin(Data_Port, cmd & 0x0F); // send lower nibble ?
	HAL_GPIO_WritePin(E_Port, E_Pin, GPIO_PIN_RESET);

	HAL_Delay(2);
}

void LCD_Init(){
	// function to initialize LCD
	HAL_Delat(20); // wait for power stabilization ?
	LCD_Command(0x33); // initialize
	LCD_Command(0x32); // set to 4-bit mode
	LCD_Command(0x28); // 2-line, 5x7 matrix (2 lines of text and 5x7 pixel font size)
	LCD_Command(0x0C); // display on and disable cursor
	LCD_Command(0x06); // increment cursor (move to right after each character is printed)
	LCD_Command(0x01); // clear display
}

void LCD_Clear(){
	// function to clear LCD
	LCD_Command(0x01); // clear display
}

void LCD_SetCursor(uint8_t row, uint8_t col){
	// sets the cursor to appropriate position based on row and col
	uint8_t addr = (row == 0 ? 0x80 : 0xC0) + col; // 0x80 -> line 1, 0xC0 -> line 2
	LCD_Command(addr);
}

void LCD_Print(char *str){
	// function to print characters to lcd screen
	while (*str){
		LCD_Data(*str++); // ? GENIUS
	}
}





