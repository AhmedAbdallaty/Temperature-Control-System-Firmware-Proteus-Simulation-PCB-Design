/*
 * *************************************************************************************************************************
 * This a Temperature Control System that uses STM32F103C8 MCU.
 * Once temperature exceeds threshold value, the fan motor is energized.
 * LM35 sensor is used to collect data from the surrounding environment. LM35 transmits data to MCU for processing.
 * TIM2_CH3 is used to send PWM signals to the motor through a motor driver (BJT 2N2222) since the motor requires much more power than what the MCU needs.
 * The system is monitored through a Virtual Monitor

* Author: Ahmed Abdallaty
  *************************************************************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>


#define TEMP_THRESHOLD_C 30.0f
#define PWM_MAX_DUTY 100

bool logging_enabled = true;   // to control UART temperature logging
float tempC = 0.0f;
uint32_t raw = 0;

void SystemClock_Config(void);
void PWM_SetDuty(uint8_t duty);
bool debounce_read(bool raw_btn);
void delayMS(uint32_t ms);

// Simple software debounce structure
typedef struct {
  bool stable_state;
  bool last_state;
  uint32_t counter;
} debounce_t;

debounce_t btn = {0};


  // @brief Simple software debounce function

bool debounce_read(bool raw_btn)
{
  if (raw_btn != btn.last_state) {
    btn.counter = 0;
    btn.last_state = raw_btn;
  } else {
    if (btn.counter < 5) btn.counter++;
    if (btn.counter >= 5) btn.stable_state = btn.last_state;
  }
  return btn.stable_state;
}


 // brief Delay in milliseconds

void delayMS(uint32_t ms)
{
  HAL_Delay(ms);
}


  // brief Set PWM duty cycle (0–100%)

/*
void PWM_SetDuty(uint8_t duty)
{
  if (duty > 100) duty = 100;

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // start PWM
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty);
}

*/

int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // start PWM
  char uart_buf[100];

  while (1)
  {
    // Read ADC value
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    // Convert raw ADC to Celsius (adjust factor as per sensor)
    tempC = (raw * 3.3f / 4095.0f) * 100.0f;

  snprintf(uart_buf, sizeof(uart_buf), "Temperature: %.2f C (raw = %lu)\r\n", tempC, raw);
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);


  // calculates how much higher the recorded temperature is above the threshold
    if (tempC >= TEMP_THRESHOLD_C) {
   /*  float span = 20.0f;
      float over = tempC - TEMP_THRESHOLD_C;
      if (over < 0.0f) over = 0.0f;
      if (over > span) over = span; //this is to know if temperature has risen to over 20 C than the threshold which will increase fan speed to duty of 100%
      uint8_t duty = (uint8_t)(50 + (over / span) * 50);

      if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
*/
     // PWM_SetDuty(100); // control the fan speed according to the calculated duty
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 100);
         HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    }
    else if (tempC < TEMP_THRESHOLD_C) {
     // PWM_SetDuty(0); // fan off only if temperature below the threshold value
    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    }

    // This is only if you want to enable logging button for printing the temperature.
    //if logging enabled, print temperature every second
    /*
   if (logging_enabled) {
    snprintf(uart_buf, sizeof(uart_buf), "Temperature: %.2f C (raw = %lu)\r\n", tempC, raw);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    }
*/

    // Read button for toggling logging
    bool raw_btn = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1); // assuming PA1 used as button
    if (debounce_read(raw_btn)) {
      if (btn.stable_state) {
        logging_enabled = !logging_enabled;
        snprintf(uart_buf, sizeof(uart_buf), "Logging %s\r\n", logging_enabled ? "ENABLED" : "DISABLED");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
      }
    }

    delayMS(500);

  }

}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
