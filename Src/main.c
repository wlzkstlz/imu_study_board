/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t HH_SPI_Send_Receive(uint16_t send_data)
{
  uint16_t result = 0;
  HAL_GPIO_WritePin(GPIOB, SPI_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, SPI_CLK_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, SPI_CS_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);

  for (uint8_t i = 0; i < 16; i++)
  {
    HAL_GPIO_WritePin(GPIOB, SPI_CLK_Pin, GPIO_PIN_RESET);
    if ((send_data & (0x01 << (15 - i))) == 0)
      HAL_GPIO_WritePin(GPIOB, SPI_MOSI_Pin, GPIO_PIN_RESET);
    else
      HAL_GPIO_WritePin(GPIOB, SPI_MOSI_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, SPI_CLK_Pin, GPIO_PIN_SET);
    if (HAL_GPIO_ReadPin(GPIOB, SPI_MISO_Pin) == GPIO_PIN_SET)
      result |= (0x01 << (15 - i));
    HAL_Delay(1);
  }

  HAL_GPIO_WritePin(GPIOB, SPI_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(1);

  return result;
}

void send_my_heart_to_u(float x, float y)
{
  const size_t data_length = 1 + sizeof(float) + sizeof(float) + 2;
  uint8_t data[data_length];
  data[0] = 0xa5;
  data[data_length - 2] = 0;
  data[data_length - 1] = 0x5a;

  memcpy(data + 1, (uint8_t *)(&x), sizeof(float));
  memcpy(data + 1 + sizeof(float), (uint8_t *)(&y), sizeof(float));

  for (size_t i = 1; i < data_length - 2; i++)
  {
    data[data_length - 2] += data[i];
  }

  HAL_UART_Transmit(&huart3, data, data_length, 0xFFFF);
}

void send_temperature(float tp)
{
  const size_t data_length = 1 + sizeof(float) + 2;
  uint8_t data[data_length];
  data[0] = 0xa5;
  data[data_length - 2] = 0;
  data[data_length - 1] = 0x5a;

  memcpy(data + 1, (uint8_t *)(&tp), sizeof(float));

  for (size_t i = 1; i < data_length - 2; i++)
  {
    data[data_length - 2] += data[i];
  }

  HAL_UART_Transmit(&huart3, data, data_length, 0xFFFF);
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
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  double t = 0.0;
  uint32_t i = 0;
  double x = 0, y = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(30);

    i++;
    t += 0.01;
    x = 16 * sin(t) * sin(t) * sin(t);
    y = 13 * cos(t) - 5 * cos(2 * t) - 2 * cos(3 * t) - cos(4 * t);

    //send_my_heart_to_u(x, y);

    uint16_t temperature = 0;

    uint16_t spi_cmd = 0;
    spi_cmd |= (0x01 << 15);
    spi_cmd |= (0x3c << 8);
    uint16_t spi_data = HH_SPI_Send_Receive(spi_cmd);
    temperature |= (spi_data & 0x00ff);

    spi_cmd = 0;
    spi_cmd |= (0x01 << 15);
    spi_cmd |= (0x3b << 8);
    spi_data = HH_SPI_Send_Receive(spi_cmd);
    temperature |= ((spi_data & 0x00ff) << 8);

    // uint8_t spi_cmd[2] = {0};
    // uint8_t spi_received[2] = {0};

    // spi_cmd[0] |= (0x01 << 7);
    // spi_cmd[0] |= 0x42;

    // HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
    // HAL_Delay(1);
    // HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    // HAL_Delay(1);
    // HAL_SPI_TransmitReceive(&hspi1, spi_cmd, spi_received, 2, 0xffff);
    // HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

    // temperature |= spi_received[1];

    // spi_cmd[0] = 0;
    // spi_cmd[1] = 0;
    // spi_received[0] = 0;
    // spi_received[1] = 0;

    // spi_cmd[0] |= (0x01 << 7);
    // spi_cmd[0] |= 0x41;

    // HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
    // HAL_Delay(1);
    // HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    // HAL_Delay(1);
    // HAL_SPI_TransmitReceive(&hspi1, spi_cmd, spi_received, 2, 0xffff);
    // HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

    // temperature |= (spi_received[1] << 8);

    float tp = (temperature / 326.8) + 25.0;

    send_temperature(tp);

    //uint8_t data[]="hello\n";
    //HAL_UART_Transmit(&huart3, data, sizeof(data), 0xFFFF);

    if (i % 40 == 0)
    {
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    }
    else if (i % 40 == 20)
    {
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    }
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
