/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CH5_OUT_Pin GPIO_PIN_5
#define CH5_OUT_GPIO_Port GPIOE
#define CH6_OUT_Pin GPIO_PIN_6
#define CH6_OUT_GPIO_Port GPIOE
#define CH5_Pin GPIO_PIN_2
#define CH5_GPIO_Port GPIOC
#define CH6_Pin GPIO_PIN_3
#define CH6_GPIO_Port GPIOC
#define LCD_CS_Pin GPIO_PIN_4
#define LCD_CS_GPIO_Port GPIOA
#define AC_CTRL_Pin GPIO_PIN_5
#define AC_CTRL_GPIO_Port GPIOA
#define AC_RES_Pin GPIO_PIN_4
#define AC_RES_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOB
#define AC_IN_Pin GPIO_PIN_7
#define AC_IN_GPIO_Port GPIOE
#define AC_IN_EXTI_IRQn EXTI9_5_IRQn
#define FAN1_CTRL_Pin GPIO_PIN_9
#define FAN1_CTRL_GPIO_Port GPIOE
#define FAN2_CTRL_Pin GPIO_PIN_11
#define FAN2_CTRL_GPIO_Port GPIOE
#define CODEC_SCL_Pin GPIO_PIN_10
#define CODEC_SCL_GPIO_Port GPIOB
#define CODEC_SDA_Pin GPIO_PIN_11
#define CODEC_SDA_GPIO_Port GPIOB
#define TS2_Pin GPIO_PIN_11
#define TS2_GPIO_Port GPIOD
#define FAN1_IN_Pin GPIO_PIN_12
#define FAN1_IN_GPIO_Port GPIOD
#define TS1_Pin GPIO_PIN_13
#define TS1_GPIO_Port GPIOD
#define FAN2_IN_Pin GPIO_PIN_14
#define FAN2_IN_GPIO_Port GPIOD
#define BUT_LED_Pin GPIO_PIN_6
#define BUT_LED_GPIO_Port GPIOC
#define LCD_BL_Pin GPIO_PIN_7
#define LCD_BL_GPIO_Port GPIOC
#define TOUCH_nCS_Pin GPIO_PIN_15
#define TOUCH_nCS_GPIO_Port GPIOA
#define TOUCH_SCK_Pin GPIO_PIN_10
#define TOUCH_SCK_GPIO_Port GPIOC
#define TOUCH_MISO_Pin GPIO_PIN_11
#define TOUCH_MISO_GPIO_Port GPIOC
#define TOUCH_MOSI_Pin GPIO_PIN_12
#define TOUCH_MOSI_GPIO_Port GPIOC
#define TOUCH_IRQ_Pin GPIO_PIN_0
#define TOUCH_IRQ_GPIO_Port GPIOD
#define PON_Pin GPIO_PIN_3
#define PON_GPIO_Port GPIOD
#define LCD_RST_Pin GPIO_PIN_6
#define LCD_RST_GPIO_Port GPIOD
#define LCD_DC_Pin GPIO_PIN_7
#define LCD_DC_GPIO_Port GPIOD
#define DSP_SCL_Pin GPIO_PIN_6
#define DSP_SCL_GPIO_Port GPIOB
#define DSP_SDA_Pin GPIO_PIN_7
#define DSP_SDA_GPIO_Port GPIOB
#define CH1_OUT_Pin GPIO_PIN_8
#define CH1_OUT_GPIO_Port GPIOB
#define CH2_OUT_Pin GPIO_PIN_9
#define CH2_OUT_GPIO_Port GPIOB
#define CH3_OUT_Pin GPIO_PIN_0
#define CH3_OUT_GPIO_Port GPIOE
#define CH4_OUT_Pin GPIO_PIN_1
#define CH4_OUT_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
typedef enum {
	STAND_BY    =  0,
	ON_REQUEST  =  1,
	SS_REQUEST  =  2,
	SS_DELAY    =  3,
	XFR_ON      =  4,
	ON          =  5,
	SB_REQUEST  =  6,
	SB_PERFORM  =  7
} eStates;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
