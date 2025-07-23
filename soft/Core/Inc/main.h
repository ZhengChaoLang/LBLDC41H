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
#include "stm32f4xx_hal.h"

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
#define RUN_LED_Pin GPIO_PIN_13
#define RUN_LED_GPIO_Port GPIOC
#define HALL_U_Pin GPIO_PIN_0
#define HALL_U_GPIO_Port GPIOA
#define HALL_V_Pin GPIO_PIN_1
#define HALL_V_GPIO_Port GPIOA
#define HALL_W_Pin GPIO_PIN_2
#define HALL_W_GPIO_Port GPIOA
#define CLR_L_HD_Pin GPIO_PIN_2
#define CLR_L_HD_GPIO_Port GPIOB
#define MCBK_Pin GPIO_PIN_12
#define MCBK_GPIO_Port GPIOB
#define MCPWM_CHNU_Pin GPIO_PIN_13
#define MCPWM_CHNU_GPIO_Port GPIOB
#define MCPWM_CHNV_Pin GPIO_PIN_14
#define MCPWM_CHNV_GPIO_Port GPIOB
#define MCPWM_CHNW_Pin GPIO_PIN_15
#define MCPWM_CHNW_GPIO_Port GPIOB
#define MCPWM_CHU_Pin GPIO_PIN_8
#define MCPWM_CHU_GPIO_Port GPIOA
#define MCPWM_CHV_Pin GPIO_PIN_9
#define MCPWM_CHV_GPIO_Port GPIOA
#define MCPWM_CHW_Pin GPIO_PIN_10
#define MCPWM_CHW_GPIO_Port GPIOA
#define RS485TX_Pin GPIO_PIN_11
#define RS485TX_GPIO_Port GPIOA
#define RS485RX_Pin GPIO_PIN_12
#define RS485RX_GPIO_Port GPIOA
#define RS485_DE_Pin GPIO_PIN_4
#define RS485_DE_GPIO_Port GPIOB
#define ENCA_Pin GPIO_PIN_6
#define ENCA_GPIO_Port GPIOB
#define ENCB_Pin GPIO_PIN_7
#define ENCB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
