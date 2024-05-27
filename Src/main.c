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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include "OneWire.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FTRIM(val, min, max) fmax(min, fmin(max, val))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern float Temp[MAXDEVICES_ON_THE_BUS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
/**
  * @brief  Retargets the C library printf function to the USART.
  */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)

#define LCD_DELAY_MS 5

uint8_t lcd_addr = (0x27 << 1);

HAL_StatusTypeDef LCD_SendInternal(uint8_t data, uint8_t flags) {
  HAL_StatusTypeDef res;
  for(;;) {
    res = HAL_I2C_IsDeviceReady(&hi2c1, lcd_addr, 1,
                                HAL_MAX_DELAY);
    if(res == HAL_OK)
      break;
  }

  uint8_t up = data & 0xF0;
  uint8_t lo = (data << 4) & 0xF0;

  uint8_t data_arr[4];
  data_arr[0] = up|flags|BACKLIGHT|PIN_EN;
  data_arr[1] = up|flags|BACKLIGHT;
  data_arr[2] = lo|flags|BACKLIGHT|PIN_EN;
  data_arr[3] = lo|flags|BACKLIGHT;

  res = HAL_I2C_Master_Transmit(&hi2c1, lcd_addr, data_arr,
                                sizeof(data_arr), HAL_MAX_DELAY);
  HAL_Delay(LCD_DELAY_MS);
  return res;
}

void LCD_SendCommand(uint8_t cmd) {
  LCD_SendInternal(cmd, 0);
}

void LCD_SendData(uint8_t data) {
  LCD_SendInternal(data, PIN_RS);
}

void LCD_Init() {
  // 4-bit mode, 2 lines, 5x7 format
  LCD_SendCommand(0b00110000);
  // display & cursor home (keep this!)
  LCD_SendCommand(0b00000010);
  // display on, underline off, blink off
  LCD_SendCommand(0b00001100);
  // clear display (optional here)
  LCD_SendCommand(0b00000001);
}

void LCD_SendString(char *str) {
  while(*str) {
    LCD_SendData((uint8_t)(*str));
    str++;
  }
}

void LCD_SendTopString(char *str) {
  LCD_SendCommand(0x80 | 0x00);
  LCD_SendString(str);
}

void LCD_SendBottomString(char *str) {
  LCD_SendCommand(0x80 | 0x40);
  LCD_SendString(str);
}

void LCD_Clean()
{
  char *str = "                ";
  LCD_SendTopString(str);
  LCD_SendBottomString(str);
}

volatile uint8_t tacho_events1 = 0;
volatile uint8_t tacho_events2 = 0;
volatile uint8_t tacho_events3 = 0;
volatile uint8_t tacho_events4 = 0;

volatile uint8_t tacho_events_per_period1 = 0;
volatile uint8_t tacho_events_per_period2 = 0;
volatile uint8_t tacho_events_per_period3 = 0;
volatile uint8_t tacho_events_per_period4 = 0;

float temp1 = 0;
float temp2 = 0;
float temp3 = 0;

void update_temperature() {
  get_Temperature();
  temp1 = FTRIM(Temp[0], 0.0, 99.9),
  temp2 = FTRIM(Temp[1], 0.0, 99.9),
  temp3 = FTRIM(Temp[2], 0.0, 99.9);
  Temp[0] = 0; Temp[1] = 0; Temp[2] = 0;
}

void display_temperature() {
  char buffer[17];
  sprintf(buffer, "%04.1f %04.1f %04.1f\1C", temp1, temp2, temp3);
  LCD_SendTopString(buffer);
}

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);

  LCD_Init();
  LCD_SendTopString   ("          RAM OK");
  LCD_SendBottomString("          ROM OK");

  get_ROMid();
  HAL_Delay(1000);

  // create a custom character (CGRAM 1, offset 8)
  LCD_SendCommand(0x40 + 8);
  LCD_SendData(0b00001100);
  LCD_SendData(0b00010010);
  LCD_SendData(0b00010010);
  LCD_SendData(0b00001100);
  LCD_SendData(0b00000000);
  LCD_SendData(0b00000000);
  LCD_SendData(0b00000000);
  LCD_SendData(0b00000000);
  LCD_SendData(0b00000000);
  LCD_Clean();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    printf("Updating temp...\r\n");
    update_temperature();
    display_temperature();
    printf("%d %d %d %d\r\n", tacho_events_per_period1, tacho_events_per_period2, tacho_events_per_period3, tacho_events_per_period4);
    HAL_Delay(100);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_3) {
    tacho_events1++;
  } else if(GPIO_Pin == GPIO_PIN_4) {
    tacho_events2++;
  } else if(GPIO_Pin == GPIO_PIN_5) {
    tacho_events3++;
  } else if(GPIO_Pin == GPIO_PIN_6) {
    tacho_events4++;
  }
}

volatile uint16_t secs = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  tacho_events_per_period1 = tacho_events1;
  tacho_events1 = 0;
  tacho_events_per_period2 = tacho_events2;
  tacho_events2 = 0;
  tacho_events_per_period3 = tacho_events3;
  tacho_events3 = 0;
  tacho_events_per_period4 = tacho_events4;
  tacho_events4 = 0;
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
