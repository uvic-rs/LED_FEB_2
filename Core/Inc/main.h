/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TRIG_OE_Pin GPIO_PIN_13
#define TRIG_OE_GPIO_Port GPIOC
#define TRIG_SEL0_Pin GPIO_PIN_14
#define TRIG_SEL0_GPIO_Port GPIOC
#define TRIG_SEL1_Pin GPIO_PIN_15
#define TRIG_SEL1_GPIO_Port GPIOC
#define BIAS_MON_LED_Pin GPIO_PIN_2
#define BIAS_MON_LED_GPIO_Port GPIOA
#define IMON_OUT_Pin GPIO_PIN_3
#define IMON_OUT_GPIO_Port GPIOA
#define BIAS_ADJUST_LED_Pin GPIO_PIN_4
#define BIAS_ADJUST_LED_GPIO_Port GPIOA
#define BOOST_ENABLE_Pin GPIO_PIN_5
#define BOOST_ENABLE_GPIO_Port GPIOA
#define SW_5V_ENABLE_Pin GPIO_PIN_6
#define SW_5V_ENABLE_GPIO_Port GPIOA
#define BIAS_ENABLE_LED_Pin GPIO_PIN_7
#define BIAS_ENABLE_LED_GPIO_Port GPIOA
#define LED_OE1_Pin GPIO_PIN_0
#define LED_OE1_GPIO_Port GPIOB
#define LED_OE2_Pin GPIO_PIN_1
#define LED_OE2_GPIO_Port GPIOB
#define LED_OE3_Pin GPIO_PIN_2
#define LED_OE3_GPIO_Port GPIOB
#define LPUART1_TX_Pin GPIO_PIN_10
#define LPUART1_TX_GPIO_Port GPIOB
#define LPUART1_RX_Pin GPIO_PIN_11
#define LPUART1_RX_GPIO_Port GPIOB
#define CS_ADC_Pin GPIO_PIN_12
#define CS_ADC_GPIO_Port GPIOB
#define SCLK_ADC_Pin GPIO_PIN_13
#define SCLK_ADC_GPIO_Port GPIOB
#define SDATA_ADC_Pin GPIO_PIN_14
#define SDATA_ADC_GPIO_Port GPIOB
#define USART1_TX_Pin GPIO_PIN_9
#define USART1_TX_GPIO_Port GPIOA
#define USART1_RX_Pin GPIO_PIN_10
#define USART1_RX_GPIO_Port GPIOA
#define IMON_RESET_Pin GPIO_PIN_11
#define IMON_RESET_GPIO_Port GPIOA
#define IMON_N_ALERT_Pin GPIO_PIN_12
#define IMON_N_ALERT_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define LED_OE5_Pin GPIO_PIN_15
#define LED_OE5_GPIO_Port GPIOA
#define uC_IN_TRIG_Pin GPIO_PIN_4
#define uC_IN_TRIG_GPIO_Port GPIOB
#define uC_OUT_TRIG_Pin GPIO_PIN_5
#define uC_OUT_TRIG_GPIO_Port GPIOB
#define LED_OE6_Pin GPIO_PIN_6
#define LED_OE6_GPIO_Port GPIOB
#define LED_OE7_Pin GPIO_PIN_7
#define LED_OE7_GPIO_Port GPIOB
#define LED_OE4_Pin GPIO_PIN_8
#define LED_OE4_GPIO_Port GPIOB
#define SW_3V3_ENABLE_Pin GPIO_PIN_9
#define SW_3V3_ENABLE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
