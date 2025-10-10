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
extern "C"
{
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

    void period_event_generator();
    void on_uac_connect();
    void on_disconnect();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPEAKER_EN_Pin GPIO_PIN_0
#define SPEAKER_EN_GPIO_Port GPIOA
#define HEADSET_OUT_EN_Pin GPIO_PIN_5
#define HEADSET_OUT_EN_GPIO_Port GPIOC
#define VBUS_DETECT_Pin GPIO_PIN_9
#define VBUS_DETECT_GPIO_Port GPIOA
#define VBUS_DETECT_EXTI_IRQn EXTI9_5_IRQn
#define LED4_Pin GPIO_PIN_2
#define LED4_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOD
#define BAT_LOW_LED_Pin GPIO_PIN_4
#define BAT_LOW_LED_GPIO_Port GPIOD
#define SYS_LED_Pin GPIO_PIN_5
#define SYS_LED_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOD
#define BT_STAT_Pin GPIO_PIN_5
#define BT_STAT_GPIO_Port GPIOB
#define BT_STAT_EXTI_IRQn EXTI9_5_IRQn
#define BT_DISABLE__Pin GPIO_PIN_6
#define BT_DISABLE__GPIO_Port GPIOB

    /* USER CODE BEGIN Private defines */

    /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
