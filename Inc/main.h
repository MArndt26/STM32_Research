/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

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
#define BUTTON_PIN_Pin GPIO_PIN_0
#define BUTTON_PIN_GPIO_Port GPIOB
#define BUTTON_PIN_EXTI_IRQn EXTI0_1_IRQn
#define LD4_BLUE_LED_Pin GPIO_PIN_8
#define LD4_BLUE_LED_GPIO_Port GPIOC
#define LD3_GREEN_LED_Pin GPIO_PIN_9
#define LD3_GREEN_LED_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */
#define START_MSG "Begin 8 Chan ADC to Micro SD"
#define SD_CARD_MOUNT_ERROR_MSG "error in mounting SD CARD..."
#define SD_CARD_MOUNT_SUCCESS_MSG "SD CARD mounted successfully..."
#define SD_CARD_TOTAL_SIZE_MSG "SD CARD Total Size: \t"
#define SD_CARD_FREE_SPACE_MSG "SD CARD Free Space: \t"
#define ADC_HEADER "ADC0 ADC1 ADC2 ADC3 ADC4 ADC5 ADC6 ADC7"
#define FILE_CREATION_MSG_PARTIAL " created and header was written"
#define DATA_COLLECTION_HALTED_MSG "Data Collection Halted.  Sending data written to serial stream\n"
#define SD_CARD_UNMOUNT_SUCCESS_MSG "SD CARD UNMOUNTED successfully..."

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
