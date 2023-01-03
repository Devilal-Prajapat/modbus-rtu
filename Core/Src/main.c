/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "modbus_crc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SLAVE_ID      17U
#define MAX_REG_READ  125U
#define MODBUS_RES_HEADER_SIZE  5U /* <1byte:id><1byte:function code> <1byte:length> <2byte:CRC>*/
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void led_cntl(uint8_t data);
void process_modbus(uint8_t *pdata, uint16_t len);
void proccess_write_holding_register(uint8_t *pdata, uint16_t len);
void proccess_read_holding_register(uint8_t *pdata, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t modbus_response_buff[RX_BUFFER_SIZE];
extern uint8_t rx_byte;
extern uint8_t rx_count;
extern uint8_t rx_length;
extern volatile uint8_t rx_complete_flag;
uint16_t holding_reg[5]={15, 30, 45, 60, 75};
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
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(rx_count >= 8 && rx_buffer[0] == SLAVE_ID)
	  {
		 rx_length = rx_count;
		 led_cntl(1);
		 process_modbus(rx_buffer, rx_length);
		 led_cntl(0);
		 rx_count = 0;
		 rx_length = 0;
#if 0
		 if(rx_buffer[0] == '$' && rx_buffer[3] == '#')
		 {
			 if(rx_buffer[1] == 0x01)
			 {
				 led_cntl(rx_buffer[2]);
				 response_buff[2] = rx_buffer[2];
				 HAL_UART_Transmit(&huart2, response_buff, 4, 100);
			 }
		 }
		 else
		 {
			 response_buff[2] = 0x03;
			 HAL_UART_Transmit(&huart2, response_buff, 4, 100);
		 }
#endif
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void led_cntl(uint8_t data)
{
	if(data)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
}

void process_modbus(uint8_t *pdata, uint16_t len)
{
	if(pdata[0] == SLAVE_ID)
	{
		switch(pdata[1])
		{
		case 0x03:  // read holding register
			proccess_read_holding_register(pdata, len);
			break;
		case 0x06: // write single holding register
			proccess_write_holding_register(pdata, len);
			break;
		}
	}
}

void proccess_read_holding_register(uint8_t *pdata, uint16_t len)
{
	uint16_t start_addr = ((pdata[2]<< 8) | pdata[3]); // start address
	uint16_t reg_count =  ((pdata[4]<< 8) | pdata[5]); // number of register to read
	uint16_t end_addr = start_addr + reg_count - 1;    // end address
	uint16_t crc =  ((pdata[6]) | pdata[7]<<8);        // received CRC
	uint16_t verify_crc = CRC16(pdata, len-2);         // calculate CRC
	if(verify_crc == crc)
	{
		if(reg_count >= 1 && (reg_count * 2) <= ((sizeof(modbus_response_buff)/sizeof(modbus_response_buff[0]))-5) && reg_count<=MAX_REG_READ)
		{
			if(end_addr <= sizeof(holding_reg)/sizeof(holding_reg[0]))
			{
				modbus_response_buff[0] = pdata[0];
				modbus_response_buff[1] = pdata[1];
				modbus_response_buff[2] = reg_count * 2;
				uint16_t response_size = MODBUS_RES_HEADER_SIZE + (2 * reg_count);
				uint8_t i = 0;
				for( i = 0; i < reg_count; i++)
				{
					modbus_response_buff[3 + (2 * i)] = holding_reg[start_addr + i] >> 8;
					modbus_response_buff[3 + (2*i +1)] = holding_reg[start_addr + i];
				}
				uint16_t check_sum = CRC16(modbus_response_buff, (3 + 2*reg_count));
				uint8_t crc_index = 3 + 2*reg_count;
				modbus_response_buff[crc_index] = check_sum;
				modbus_response_buff[crc_index + 1] = check_sum>>8;
				HAL_UART_Transmit(&huart2, modbus_response_buff, response_size, 100);
			}
		}
	}
}

void proccess_write_holding_register(uint8_t *pdata, uint16_t len)
{
	uint16_t addr = ((pdata[2]<< 8) | pdata[3]);
	uint16_t data_to_write = ((pdata[4]<< 8) | pdata[5]);
	uint16_t crc =  ((pdata[6]) | pdata[7] << 8);
	uint16_t verify_crc = CRC16(pdata, len-2);
	if(verify_crc == crc)
	{
		if(addr < 5)
		{
			holding_reg[addr] = data_to_write;
			modbus_response_buff[0] = pdata[0];
			modbus_response_buff[1] = pdata[1];
			modbus_response_buff[2] = pdata[2];
			modbus_response_buff[3] = pdata[3];
			modbus_response_buff[4] = holding_reg[addr]>>8;
			modbus_response_buff[5] = holding_reg[addr];
			uint16_t check_sum = CRC16(modbus_response_buff, 6);
			modbus_response_buff[6] = check_sum;
			modbus_response_buff[7] = check_sum>>8;
			HAL_UART_Transmit(&huart2, modbus_response_buff, 8, 100);
		}
	}
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
