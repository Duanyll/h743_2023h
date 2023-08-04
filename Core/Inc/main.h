/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FPGA_CS_Pin GPIO_PIN_5
#define FPGA_CS_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_2
#define KEY1_GPIO_Port GPIOC
#define ADC_SCK_Pin GPIO_PIN_6
#define ADC_SCK_GPIO_Port GPIOE
#define KEY0_Pin GPIO_PIN_0
#define KEY0_GPIO_Port GPIOC
#define FPGA_RESET_Pin GPIO_PIN_1
#define FPGA_RESET_GPIO_Port GPIOC
#define ADC_DIO_Pin GPIO_PIN_3
#define ADC_DIO_GPIO_Port GPIOC
#define ADC_SLEEP_Pin GPIO_PIN_4
#define ADC_SLEEP_GPIO_Port GPIOC
#define PLL_ENABLE_Pin GPIO_PIN_2
#define PLL_ENABLE_GPIO_Port GPIOB
#define PLL_SDI_Pin GPIO_PIN_10
#define PLL_SDI_GPIO_Port GPIOE
#define ADC_CS_Pin GPIO_PIN_5
#define ADC_CS_GPIO_Port GPIOC
#define PLL_CS_Pin GPIO_PIN_7
#define PLL_CS_GPIO_Port GPIOE
#define PLL_SCK_Pin GPIO_PIN_11
#define PLL_SCK_GPIO_Port GPIOE
#define FPGA_BUSY_Pin GPIO_PIN_2
#define FPGA_BUSY_GPIO_Port GPIOA
#define FPGA_READST_Pin GPIO_PIN_0
#define FPGA_READST_GPIO_Port GPIOB
#define FPGA_SDA_Pin GPIO_PIN_8
#define FPGA_SDA_GPIO_Port GPIOE
#define FPGA_SCLK_Pin GPIO_PIN_13
#define FPGA_SCLK_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
