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
#include "stm32g0xx_hal.h"

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
#define USB_I2C_RES_Pin GPIO_PIN_0
#define USB_I2C_RES_GPIO_Port GPIOA
#define MB_FAN_TACH_Pin GPIO_PIN_1
#define MB_FAN_TACH_GPIO_Port GPIOA
#define MB_PSON_Pin GPIO_PIN_2
#define MB_PSON_GPIO_Port GPIOA
#define MB_PWROK_Pin GPIO_PIN_3
#define MB_PWROK_GPIO_Port GPIOA
#define PWR_SW_Pin GPIO_PIN_4
#define PWR_SW_GPIO_Port GPIOA
#define RST_SW_Pin GPIO_PIN_5
#define RST_SW_GPIO_Port GPIOA
#define MB_PWROKA6_Pin GPIO_PIN_6
#define MB_PWROKA6_GPIO_Port GPIOA
#define MCU_ATTACH_IN_Pin GPIO_PIN_7
#define MCU_ATTACH_IN_GPIO_Port GPIOA
#define U3_FAN_I2C2_SCL_Pin GPIO_PIN_11
#define U3_FAN_I2C2_SCL_GPIO_Port GPIOA
#define U3_FAN_I2C2_SDA_Pin GPIO_PIN_12
#define U3_FAN_I2C2_SDA_GPIO_Port GPIOA
#define MB_BITCH_Pin GPIO_PIN_3
#define MB_BITCH_GPIO_Port GPIOB
#define MB_STATUS_LED_Pin GPIO_PIN_4
#define MB_STATUS_LED_GPIO_Port GPIOB
#define MCU_HOS_ON_Pin GPIO_PIN_6
#define MCU_HOS_ON_GPIO_Port GPIOB
#define FAN_I2C1_SDA_Pin GPIO_PIN_7
#define FAN_I2C1_SDA_GPIO_Port GPIOB
#define FAN_I2C1_SCL_Pin GPIO_PIN_8
#define FAN_I2C1_SCL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
