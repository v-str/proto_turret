/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

#include "stm32f4xx_nucleo.h"

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
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define M1_EN_Pin GPIO_PIN_4
#define M1_EN_GPIO_Port GPIOA
#define M1_STEP_Pin GPIO_PIN_8
#define M1_STEP_GPIO_Port GPIOA
#define M1_DIR_Pin GPIO_PIN_9
#define M1_DIR_GPIO_Port GPIOA
#define M2_STEP_Pin GPIO_PIN_10
#define M2_STEP_GPIO_Port GPIOA
#define M2_DIR_Pin GPIO_PIN_11
#define M2_DIR_GPIO_Port GPIOA
#define LASER_Pin GPIO_PIN_12
#define LASER_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* --- User LED (Nucleo-F446RE: LD2 on PA5) --- */
#define LD1_Pin                   LED2_PIN
#define LD1_GPIO_Port             LED2_GPIO_PORT
#define LD2_Pin                   LED2_PIN
#define LD2_GPIO_Port             LED2_GPIO_PORT
#define LD3_Pin                   LED2_PIN
#define LD3_GPIO_Port             LED2_GPIO_PORT

#define USER_Btn_Pin              USER_BUTTON_PIN
#define USER_Btn_GPIO_Port        USER_BUTTON_GPIO_PORT

#define USB_PowerSwitchOn_Pin     GPIO_PIN_5
#define USB_PowerSwitchOn_GPIO_Port GPIOB
#define USB_OverCurrent_Pin       GPIO_PIN_4
#define USB_OverCurrent_GPIO_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
