/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f2xx_hal.h"

#include "stm32f2xx_ll_dma.h"
#include "stm32f2xx_ll_usart.h"
#include "stm32f2xx_ll_rcc.h"
#include "stm32f2xx_ll_bus.h"
#include "stm32f2xx_ll_cortex.h"
#include "stm32f2xx_ll_system.h"
#include "stm32f2xx_ll_utils.h"
#include "stm32f2xx_ll_pwr.h"
#include "stm32f2xx_ll_gpio.h"

#include "stm32f2xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
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
extern uint32_t sys_get_ms(void);
extern bool lock_debug(bool lock, uint32_t timeout_ms);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ESP_EN_Pin GPIO_PIN_13
#define ESP_EN_GPIO_Port GPIOC
#define ESP_IO0_Pin GPIO_PIN_14
#define ESP_IO0_GPIO_Port GPIOC
#define ADC_3V3_Pin GPIO_PIN_2
#define ADC_3V3_GPIO_Port GPIOC
#define ADC_1V8_Pin GPIO_PIN_3
#define ADC_1V8_GPIO_Port GPIOC
#define ADC_VBATRF_Pin GPIO_PIN_0
#define ADC_VBATRF_GPIO_Port GPIOA
#define ADC_VBAT_Pin GPIO_PIN_3
#define ADC_VBAT_GPIO_Port GPIOA
#define ADC_5V_Pin GPIO_PIN_4
#define ADC_5V_GPIO_Port GPIOA
#define ADC_VSYS_Pin GPIO_PIN_5
#define ADC_VSYS_GPIO_Port GPIOA
#define ADC_VIN_Pin GPIO_PIN_6
#define ADC_VIN_GPIO_Port GPIOA
#define PRE_PWR_Pin GPIO_PIN_0
#define PRE_PWR_GPIO_Port GPIOB
#define ISO_IN1_Pin GPIO_PIN_1
#define ISO_IN1_GPIO_Port GPIOB
#define ISO_IN2_Pin GPIO_PIN_2
#define ISO_IN2_GPIO_Port GPIOB
#define ISO_IN3_Pin GPIO_PIN_11
#define ISO_IN3_GPIO_Port GPIOF
#define ISO_IN4_Pin GPIO_PIN_12
#define ISO_IN4_GPIO_Port GPIOF
#define IN_COM1_Pin GPIO_PIN_13
#define IN_COM1_GPIO_Port GPIOF
#define IN_COM2_Pin GPIO_PIN_14
#define IN_COM2_GPIO_Port GPIOF
#define BUTTON_Pin GPIO_PIN_15
#define BUTTON_GPIO_Port GPIOF
#define ALARM_IN_Pin GPIO_PIN_1
#define ALARM_IN_GPIO_Port GPIOG
#define FAULT_IN_Pin GPIO_PIN_7
#define FAULT_IN_GPIO_Port GPIOE
#define RELAY_NO_Pin GPIO_PIN_8
#define RELAY_NO_GPIO_Port GPIOE
#define RELAY_NC_Pin GPIO_PIN_9
#define RELAY_NC_GPIO_Port GPIOE
#define INPUT1_Pin GPIO_PIN_10
#define INPUT1_GPIO_Port GPIOE
#define INPUT2_Pin GPIO_PIN_11
#define INPUT2_GPIO_Port GPIOE
#define INPUT3_Pin GPIO_PIN_12
#define INPUT3_GPIO_Port GPIOE
#define RELAY_COM_Pin GPIO_PIN_13
#define RELAY_COM_GPIO_Port GPIOE
#define STATUS_Pin GPIO_PIN_14
#define STATUS_GPIO_Port GPIOE
#define EN_STWD_Pin GPIO_PIN_15
#define EN_STWD_GPIO_Port GPIOE
#define ESP_RX_Pin GPIO_PIN_10
#define ESP_RX_GPIO_Port GPIOB
#define ESP_TX_Pin GPIO_PIN_11
#define ESP_TX_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define LED_DONE_Pin GPIO_PIN_8
#define LED_DONE_GPIO_Port GPIOD
#define LED_ERROR_Pin GPIO_PIN_9
#define LED_ERROR_GPIO_Port GPIOD
#define LED_BUSY_Pin GPIO_PIN_10
#define LED_BUSY_GPIO_Port GPIOD
#define MODE2_Pin GPIO_PIN_6
#define MODE2_GPIO_Port GPIOC
#define MODE1_Pin GPIO_PIN_7
#define MODE1_GPIO_Port GPIOC
#define GSM_PWRKEY_Pin GPIO_PIN_8
#define GSM_PWRKEY_GPIO_Port GPIOC
#define GSM_RESET_Pin GPIO_PIN_9
#define GSM_RESET_GPIO_Port GPIOC
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define GSM_RX_Pin GPIO_PIN_10
#define GSM_RX_GPIO_Port GPIOC
#define GSM_TX_Pin GPIO_PIN_11
#define GSM_TX_GPIO_Port GPIOC
#define RS485_TX_Pin GPIO_PIN_12
#define RS485_TX_GPIO_Port GPIOC
#define BT_IN_Pin GPIO_PIN_0
#define BT_IN_GPIO_Port GPIOD
#define RS485_RX_Pin GPIO_PIN_2
#define RS485_RX_GPIO_Port GPIOD
#define BUZZ_Pin GPIO_PIN_3
#define BUZZ_GPIO_Port GPIOD
#define ESP_RXD5_Pin GPIO_PIN_5
#define ESP_RXD5_GPIO_Port GPIOD
#define ESP_TXD6_Pin GPIO_PIN_6
#define ESP_TXD6_GPIO_Port GPIOD
#define W_ENET_RST_Pin GPIO_PIN_12
#define W_ENET_RST_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
