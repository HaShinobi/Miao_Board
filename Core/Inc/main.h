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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
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

/* Globals owned by main.c and shared with the application modules.
   Declared here so every consumer agrees on the type - hand-written extern
   lines in each .c cannot be checked against the definition. */

/* CAN buses. FDCAN1 = spools (CANopen) + forming motors (CanMessage_t),
   125 kbps. FDCAN2 = SDM8070 servo, 1 Mbps. */
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

extern volatile uint32_t g_build_tag;        // bump every change; check in Ozone
extern volatile uint8_t  program_state;      // 0 = manual bench, 1 = arm sequence
extern volatile uint8_t  last_program_state;

extern volatile uint8_t  pump_state;         // bench pump command
extern volatile uint8_t  last_pump_state;
extern volatile float    volume_pumped_mL;   // shared volume integral

extern const float r_motor;                  // spool radius [mm]

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ValveOut_Pin GPIO_PIN_0
#define ValveOut_GPIO_Port GPIOA
#define ValveIn_Pin GPIO_PIN_2
#define ValveIn_GPIO_Port GPIOA
#define PumpOut_Pin GPIO_PIN_9
#define PumpOut_GPIO_Port GPIOE
#define PumpIn_Pin GPIO_PIN_13
#define PumpIn_GPIO_Port GPIOE

/* USART2 RS485 DE/RE — flow sensor (RS485-U2, PD4) */
#define RS485_DE_RE_2_Pin       GPIO_PIN_4
#define RS485_DE_RE_2_GPIO_Port GPIOD

/* USART3 RS485 DE/RE — pressure sensor (RS485-U3, PB14) */
#define RS485_DE_RE_3_Pin       GPIO_PIN_14
#define RS485_DE_RE_3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
