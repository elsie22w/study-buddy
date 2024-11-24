/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "stdio.h"
#include "i2c-lcd.h"
#include "custom_chars.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char msg[20];
char receivedButton;
int curScreen = TIME_SETUP;

//uint8_t CHAR_CLOCK = 0x00;
//int MENU = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


//  LCD_Init(); // initialize LCD
//  LCD_Command(0x80);
//  LCD_Print("Hello, World");


  setRTCTimer(&hrtc, 50, 0, 0); // sets alarm

  LCD_Init();
  LCD_Clear();
  LCD_Command(0x0F); // Display on, cursor on, blinking cursor on
  LCD_Command(0x80);

  HAL_Delay(50);
  LCD_Menu(curScreen);
  curScreen = DATE_SETUP;
  LCD_Menu(curScreen);


  //  LCD_CreateChar(0, clock); //
//  LCD_Data(CHAR_CLOCK);
//  LCD_Print("Hello!");



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  curScreen = MAIN;
//	  printCurrentTime();
	  if (LCD_Menu(curScreen) == SET_TIMER){
		  curScreen = SET_TIMER;
	  } else {
		  curScreen = MAIN;
		  LCD_Menu(curScreen);
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x1;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
//	_HAL_RCC_USART1_CLK_ENABLE();

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600; // org: 115200
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void printCurrentTime(){
	// prints the current time to the console (debugging purposes)
	RTC_TimeTypeDef currentTime;
	RTC_DateTypeDef currentDate;

	HAL_RTC_GetTime(&hrtc, &currentTime, FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currentDate, FORMAT_BIN);

	sprintf(msg, "%02d:%02d:%02d", currentTime.Hours, currentTime.Minutes, currentTime.Seconds);

	    // Send the time to the console for debugging
	    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	    // Update the LCD
//	    LCD_Command(0x80);   // Move cursor to the first line, first column
//	    LCD_Print("Time:");  // Print a static label
//	    LCD_Command(0xC0);   // Move cursor to the second line
//	    LCD_Print(msg);      // Print the current time
}

int LCD_Menu(int menu){
	if (menu == MAIN){
		char timeBuffer[10];    // Buffer for "HH:MM" (5 chars + 1 null terminator)
		char dateBuffer[10];
		char menuLine1[20];    // Buffer for full LCD line (adjust size as needed)
		char menuLine2[20];
		int pos = 0;

		RTC_TimeTypeDef currentTime;
		RTC_DateTypeDef currentDate;

						    // Get the current time and date
		HAL_RTC_GetTime(&hrtc, &currentTime, FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &currentDate, FORMAT_BIN);

		//				    // Format the time as "HH:MM"
		//		LCD_Data(CHAR_CLOCK);
		snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d", currentTime.Hours, currentTime.Minutes);
		snprintf(dateBuffer, sizeof(dateBuffer), "%02d-%02d", currentDate.Month, currentDate.Date);
						    // Construct the menu string for the first line: "T HH:MM    X"
		snprintf(menuLine1, sizeof(menuLine1), "C%s    MSEWTA", timeBuffer);
		snprintf(menuLine2, sizeof(menuLine2), "%s  XHH:MM:SS", dateBuffer);

						     //Display the menu on the LCD
		LCD_Command(0x80);     // Move cursor to the first line, first column
		LCD_Print(menuLine1);  // Print the formatted menu string
		LCD_Command(0xC0);
		LCD_Print(menuLine2);


		//		LCD_Print("T");
		LCD_Command(0x80);

		while (true){
			if (HAL_UART_Receive(&huart1, (uint8_t*)&receivedButton, 1, HAL_MAX_DELAY) == HAL_OK) {
				// Format received button for UART transmission
				// Handle received button for LCD
				if (receivedButton == 'L') {
					LCD_Command(0x10); // Move cursor left
					--pos;
				}
				else if (receivedButton == 'R') {
					LCD_Command(0x14); // Move cursor right
					++pos;
				} else if ((pos == 14) && (receivedButton == '0')){
//					LCD_Menu(SET_TIMER);
					return SET_TIMER;
					break;
				}
			}
		}
	} else if (menu == TIME_SETUP){
		int pos = 0;
		int array[5];

		LCD_Command(0x80);
		LCD_Print("Current Time:");
		LCD_Command(0xC0);
		LCD_Print("HH:MM Y");
		LCD_Command(0xC0);
		while (true){
			if (HAL_UART_Receive(&huart1, (uint8_t*)&receivedButton, 1, HAL_MAX_DELAY) == HAL_OK) {
					// Format received button for UART transmission
					// Handle received button for LCD
					if (receivedButton == 'L') {
						LCD_Command(0x10); // Move cursor left
						--pos;
					}
					else if (receivedButton == 'R') {
						LCD_Command(0x14); // Move cursor right
						++pos;
					}
					else {
						if ((pos == 6) && (receivedButton == '0')){
							LCD_Clear();
							sprintf(msg, "%d\r\n", array[0]);
							HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

							RTC_TimeTypeDef sTime;

							    // Set time to 15:30:00 (3:30 PM)
							sTime.Hours = array[0]*10+array[1];    // 3 PM (in 24-hour format)
							sTime.Minutes = array[3]*10+array[4];  // 30 minutes
							sTime.Seconds = 0;   // 0 seconds
							sTime.TimeFormat = RTC_HOURFORMAT_24;  // Set to 24-hour format
							sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;  // No daylight saving time
							sTime.StoreOperation = RTC_STOREOPERATION_RESET;  // No store operation

							    // Set the time using HAL_RTC_SetTime function
							if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
								sprintf(msg, "Error setting time\r\n");
								    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								Error_Handler();}
							return MAIN;
							break;
						} else if (pos == 0 || pos == 1 || pos == 3 || pos == 4){
							sprintf(msg, "%c", receivedButton); // Only send the character to LCD
							LCD_Print(msg); // Display the received character
							array[pos] = receivedButton - '0';
							++pos;
						}
					}
					sprintf(msg, "%d\r\n", array[0]);
					HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);


				}
		}
	} else if (menu == DATE_SETUP){
		int pos = 0;
		uint8_t array[5];

		LCD_Command(0x80);
		LCD_Print("Current Date:");
		LCD_Command(0xC0);
		LCD_Print("MM-DD Y");
		LCD_Command(0xC0);
		while (true){
			if (HAL_UART_Receive(&huart1, (uint8_t*)&receivedButton, 1, HAL_MAX_DELAY) == HAL_OK) {
				if (receivedButton == 'L') {
					LCD_Command(0x10); // Move cursor left
					--pos;
							}
				else if (receivedButton == 'R') {
					LCD_Command(0x14); // Move cursor right
					++pos;
						}
				else {
					if ((pos == 6) && (receivedButton == '0')){
						LCD_Clear();
						RTC_DateTypeDef sDate;

						sDate.WeekDay = RTC_WEEKDAY_MONDAY;
						sDate.Month = array[0]*10 + array[1];
						sDate.Date = array[3]*10 + array[4];
						sDate.Year = 0x0;

						if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
							Error_Handler();}
						return MAIN;
						break;
					} else if (pos == 0 || pos == 1 || pos == 3 || pos == 4){
						sprintf(msg, "%c", receivedButton); // Only send the character to LCD
						LCD_Print(msg); // Display the received character
						array[pos] = receivedButton - '0';
						++pos;
					}
				}
				sprintf(msg, "%d\r\n", pos);
				HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);


			}
		}
	} else if (menu == SET_TIMER){
		int pos = 0;
		int array[8];

		LCD_Clear();
		LCD_Command(0x80);
		LCD_Print("Timer Length:");
		LCD_Command(0xC0);
		LCD_Print("HH:MM:SS Y");
		LCD_Command(0xC0);
		while (true){
			if (HAL_UART_Receive(&huart1, (uint8_t*)&receivedButton, 1, HAL_MAX_DELAY) == HAL_OK) {
							// Format received button for UART transmission
							// Handle received button for LCD
							if (receivedButton == 'L') {
								LCD_Command(0x10); // Move cursor left
								--pos;
							}
							else if (receivedButton == 'R') {
								LCD_Command(0x14); // Move cursor right
								++pos;
							}
							else {
								if ((pos == 9) && (receivedButton == '0')){
									LCD_Clear();
									setRTCTimer(&hrtc, array[6]*10+array[7], array[3]*10+array[4], array[0]*10+array[1]);
									return MAIN;
									break;
								} else if (pos == 0 || pos == 1 || pos == 3 || pos == 4 || pos == 6 || pos == 7){
									sprintf(msg, "%c", receivedButton); // Only send the character to LCD
									LCD_Print(msg); // Display the received character
									array[pos] = receivedButton - '0';
									++pos;
								}
							}
							sprintf(msg, "%d\r\n", pos);
							HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);


						}
				}
	}
	return 1;
}


void setRTCAlarm(RTC_HandleTypeDef *hrtc, int sec, int min, int hour){
	// sets an alarm to specified time
	RTC_AlarmTypeDef sAlarm;
	RTC_TimeTypeDef currentTime;

	HAL_RTC_GetTime(hrtc, &currentTime, FORMAT_BIN); // gets current time


	sAlarm.AlarmTime.Seconds = sec;
	sAlarm.AlarmTime.Minutes = min;
	sAlarm.AlarmTime.Hours = hour;
	if(sAlarm.AlarmTime.Seconds >= 60){ // handles second overflow
		sAlarm.AlarmTime.Seconds -= 60;
		++sAlarm.AlarmTime.Minutes;
	}
	if(sAlarm.AlarmTime.Minutes >= 60){ // handles minute overflow
			sAlarm.AlarmTime.Minutes -= 60;
			++sAlarm.AlarmTime.Hours;
	}
	if(sAlarm.AlarmTime.Hours >= 24){ // handles hour overflow
			sAlarm.AlarmTime.Hours -= 24;
	}

	sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY; // ignores date
	sAlarm.Alarm = RTC_ALARM_A;

	while(HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, FORMAT_BIN) != HAL_OK){}
	// keeps on setting alarm until it works
}

void setRTCTimer(RTC_HandleTypeDef *hrtc, int sec, int min, int hour){
	// sets an alarm to specified time
	RTC_AlarmTypeDef sAlarm;
	RTC_TimeTypeDef currentTime;

	HAL_RTC_GetTime(hrtc, &currentTime, RTC_FORMAT_BIN); // gets current time


	// Add seconds, minutes, and hours to the current time
	int new_seconds = currentTime.Seconds + sec;
	int new_minutes = currentTime.Minutes + min + (new_seconds / 60);
	int new_hours = currentTime.Hours + hour + (new_minutes / 60);

	    // Handle overflow
	new_seconds %= 60;
	new_minutes %= 60;
	new_hours %= 24;

	    // Set the alarm time
	sAlarm.AlarmTime.Seconds = new_seconds;
	sAlarm.AlarmTime.Minutes = new_minutes;
	sAlarm.AlarmTime.Hours = new_hours;
	sAlarm.AlarmTime.SubSeconds = 0;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;

	    // Configure the alarm
	sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	sAlarm.AlarmDateWeekDay = 0x1; // Current day (placeholder)
	sAlarm.Alarm = RTC_ALARM_A;

	    // Set the alarm and enable interrupt
	if (HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // toggles pin on alarm
//	LCD_Clear();

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
