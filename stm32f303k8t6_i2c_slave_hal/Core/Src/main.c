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
#include "usart.h"
#include "gpio.h"

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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Флаги для 10 команд
volatile uint8_t command_received[10] = {0};

// Определите буферы для приема и передачи данных
uint8_t rxBuffer[2];	// Буфер для приема данных
uint8_t txBuffer[2];  // Буфер для передачи данных

// Измеренное значение АЦП
volatile uint16_t ADC_30V = 0;
volatile uint16_t ADC_08V = 0;
volatile uint16_t ADC_PAD = 0;
volatile uint16_t ADC_OTR = 0;
volatile uint16_t RES     = 0;
volatile uint16_t V_BAT   = 0;

#define tV_25   1.43f      // Напряжение (в вольтах) на датчике при температуре 25 °C.
#define tSlope  0.0043f    // Изменение напряжения (в вольтах) при изменении температуры на градус.
#define Vref    3.3f       // Образцовое напряжение АЦП (в вольтах).

float TEMP;
float POWER;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Запуск I2C slave на прием данных
  if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK)
    {
      /* Transfer error in reception process */
      Error_Handler();
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  	  // приём данных
	  	  if (HAL_I2C_Slave_Receive_IT(&hi2c1, rxBuffer, sizeof(rxBuffer)) != HAL_OK)
	      {
	          // Ошибка приема
	          Error_Handler();
	      }

	  	  if (command_received[0] == 1)  // Калибровка АЦП отр ADC2
	  	  {
		  	  HAL_ADCEx_Calibration_Start(&hadc2, 2);
	  		  command_received[0] = 0;
	  	  }

	  	  if (command_received[1] == 1) // АЦП отр ADC2
	  	  {
	  		 HAL_ADCEx_InjectedStart(&hadc2); // Запуск АЦП
	  		 HAL_ADC_PollForConversion(&hadc2,100); //дождёмся окончания преобразований
	  		 ADC_OTR = HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_2);
	  		 txBuffer[0] = (int16_t) ADC_OTR;
	  		 HAL_ADCEx_InjectedStop(&hadc2); //остановим преобразования
	  	  	 command_received[1] = 0;
	  	  }

	  	  if (command_received[2] == 1) // Калибровка АЦП пад
	  	  {
	  		 HAL_ADCEx_Calibration_Start(&hadc2, 1);
	  	  	 command_received[2] = 0;
	  	  }

	  	  if (command_received[3] == 1) // АЦП пад
	  	  {
	  		HAL_ADCEx_InjectedStart(&hadc2);
	  		HAL_ADC_PollForConversion(&hadc2,100); //дождёмся окончания преобразований
	  		ADC_PAD = HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_1);
	  		txBuffer[0] = (int16_t) ADC_PAD;
	  		HAL_ADCEx_InjectedStop(&hadc2); //остановим преобразования
	  		command_received[3] = 0;
	  	  }

	  	  if (command_received[4] == 1) // Калибровка АЦП 30В
	  	  {
	  		  HAL_ADCEx_Calibration_Start(&hadc1, 1);
	  	  	  command_received[4] = 0;
	  	  }

	  	  if (command_received[5] == 1) // АЦП 30 В
	  	  {
	  		HAL_ADCEx_InjectedStart(&hadc1);
	  		HAL_ADC_PollForConversion(&hadc1,100); //дождёмся окончания преобразований
	  		ADC_30V = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1);
	  		txBuffer[0] = (int16_t) ADC_30V;
	  		HAL_ADCEx_InjectedStop(&hadc1); //остановим преобразования
	  		command_received[5] = 0;
	  	  }

	  	  if (command_received[6] == 1) // Калибровка АЦП 8 В
	  	  {
	  		 HAL_ADCEx_Calibration_Start(&hadc1, 2);
	  		 command_received[6] = 0;
	  	  }

	  	  if (command_received[7] == 1) // АЦП 8 В
	  	  {
	  		HAL_ADCEx_InjectedStart(&hadc1);
	  		HAL_ADC_PollForConversion(&hadc1,100); //дождёмся окончания преобразований
	  		ADC_08V = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2);
	  		txBuffer[0] = (int16_t) ADC_08V;
	  		HAL_ADCEx_InjectedStop(&hadc1); //остановим преобразования
	  	  	command_received[7] = 0;
	  	  }

	  	  if (command_received[8] == 1) // Температура
	  	  {
	  		HAL_ADCEx_InjectedStart(&hadc1);
	  		HAL_ADC_PollForConversion(&hadc1,100);
	  		RES = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
	  		TEMP = (float) RES/4096*Vref;   // Напряжение в вольтах на датчике.
	  		TEMP = (tV_25-TEMP)/tSlope + 25;   // Температура в градусах.
	  		RES = (int16_t) TEMP;
	  		txBuffer[0] = (int16_t) TEMP;
	  		HAL_ADCEx_InjectedStop(&hadc1); //остановим преобразования
	  	  	command_received[8] = 0;
	  	  }

	  	  if (command_received[9] == 1) // Мощность
	  	  {
	  		HAL_ADCEx_InjectedStart(&hadc1);
	  		HAL_ADC_PollForConversion(&hadc1,100);
	  		V_BAT = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);
	  		POWER = (float) V_BAT/4096*Vref; // Напряжение в вольтах
	  		txBuffer[0] = (int16_t) POWER;
	  		HAL_ADCEx_InjectedStop(&hadc1); //остановим преобразования
	  	  	command_received[9] = 0;
	  	  }

	  	  if (HAL_I2C_Slave_Transmit_IT(&hi2c1, txBuffer, sizeof(txBuffer)) != HAL_OK)
	  	  {
	  	    // Ошибка отправки
	  	    Error_Handler();
	  	  }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* Обработчик прерывания по приему данных по I2C */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // Данные успешно приняты
    // Здесь можно обработать принятые данные
	switch (rxBuffer[0])
		        {
		            case 0x0A: // Калибровка АЦП отр
		                command_received[0] = 1;
		                rxBuffer[0] = 0;
		                break;
		            case 0x05: // АЦП отр
		                command_received[1] = 1;
		                rxBuffer[0] = 0;
		                break;
		            case 0x0B: // Калибровка АЦП пад
		                command_received[2] = 1;
		                rxBuffer[0] = 0;
		                break;
		            case 0x06: // АЦП пад
		                command_received[3] = 1;
		                rxBuffer[0] = 0;
		                break;
		            case 0x0C: // Калибровка АЦП 30В
		                command_received[4] = 1;
		                rxBuffer[0] = 0;
		                break;
		            case 0x07: // АЦП 30 В
		                command_received[5] = 1;
		                rxBuffer[0] = 0;
		                break;
		            case 0x0D: // Калибровка АЦП 8 В
		                command_received[6] = 1;
		                rxBuffer[0] = 0;
		                break;
		            case 0x08: // АЦП 8 В
		                command_received[7] = 1;
		                rxBuffer[0] = 0;
		                break;
		            case 0x0E: // Температура
		                command_received[8] = 1;
		                rxBuffer[0] = 0;
		                break;
		            case 0x0F: // Мощность
		                command_received[9] = 1;
		                rxBuffer[0] = 0;
		                break;
		            default:
		                break;
		        }
	/*
	 // Перезапуск приема
	    if (HAL_I2C_Slave_Receive_IT(hi2c, rxBuffer, sizeof(rxBuffer)) != HAL_OK)
	    {
	        // Ошибка приема
	        Error_Handler();
	    }
	    */
}



/* Обработчик прерывания по передаче данных по I2C */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // Данные успешно переданы
    // Здесь можно обновить данные для передачи, если необходимо
	// Очистка буфера
	// Очистка буфера
	for (int i = 0; i < sizeof(txBuffer); i++)
	{
	    txBuffer[i] = 0;
	}
	/*
    // Перезапуск передачи
    if (HAL_I2C_Slave_Transmit_IT(hi2c, txBuffer, sizeof(txBuffer)) != HAL_OK)
    {
        // Ошибка передачи
        Error_Handler();
    }
    */
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
