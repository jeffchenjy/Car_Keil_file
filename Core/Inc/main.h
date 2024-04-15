/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI4_15_IRQn
#define Sharp_left_ADC_Pin GPIO_PIN_0
#define Sharp_left_ADC_GPIO_Port GPIOC
#define Sharp_right_ADC_Pin GPIO_PIN_1
#define Sharp_right_ADC_GPIO_Port GPIOC
#define back_PING_Sensor_Trig_Pin GPIO_PIN_2
#define back_PING_Sensor_Trig_GPIO_Port GPIOC
#define DHT22_Sensor_Pin GPIO_PIN_3
#define DHT22_Sensor_GPIO_Port GPIOC
#define Rasp_Pi_RX_Pin GPIO_PIN_0
#define Rasp_Pi_RX_GPIO_Port GPIOA
#define Rasp_Pi_TX_Pin GPIO_PIN_1
#define Rasp_Pi_TX_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define car_ain1_Pin GPIO_PIN_4
#define car_ain1_GPIO_Port GPIOC
#define car_ain2_Pin GPIO_PIN_5
#define car_ain2_GPIO_Port GPIOC
#define front_PING_Sensor_Trig_Pin GPIO_PIN_2
#define front_PING_Sensor_Trig_GPIO_Port GPIOB
#define UR_TX_DFPlayer_RX_Pin GPIO_PIN_10
#define UR_TX_DFPlayer_RX_GPIO_Port GPIOB
#define UR_RX_DFPlayer_TX_Pin GPIO_PIN_11
#define UR_RX_DFPlayer_TX_GPIO_Port GPIOB
#define servo_motor_Pin GPIO_PIN_14
#define servo_motor_GPIO_Port GPIOB
#define car_bin1_Pin GPIO_PIN_6
#define car_bin1_GPIO_Port GPIOC
#define car_bin2_Pin GPIO_PIN_7
#define car_bin2_GPIO_Port GPIOC
#define back_PING_Sensor_Echo_Pin GPIO_PIN_8
#define back_PING_Sensor_Echo_GPIO_Port GPIOA
#define BT_TX_Pin GPIO_PIN_9
#define BT_TX_GPIO_Port GPIOA
#define BT_RX_Pin GPIO_PIN_10
#define BT_RX_GPIO_Port GPIOA
#define front_PING_Sensor_Echo_Pin GPIO_PIN_11
#define front_PING_Sensor_Echo_GPIO_Port GPIOA
#define car_stby_Pin GPIO_PIN_12
#define car_stby_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define MH_sensor_right_Pin GPIO_PIN_11
#define MH_sensor_right_GPIO_Port GPIOC
#define MH_sensor_right_EXTI_IRQn EXTI4_15_IRQn
#define MH_sensor_left_Pin GPIO_PIN_12
#define MH_sensor_left_GPIO_Port GPIOC
#define MH_sensor_left_EXTI_IRQn EXTI4_15_IRQn
#define I2C1_SCL_LCD_Pin GPIO_PIN_6
#define I2C1_SCL_LCD_GPIO_Port GPIOB
#define I2C1_SDA_LCD_Pin GPIO_PIN_7
#define I2C1_SDA_LCD_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
