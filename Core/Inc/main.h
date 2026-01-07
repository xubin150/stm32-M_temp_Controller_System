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
#include "bsp_max31855_driver.h"
#include "bsp_pid.h"
#include "bsp_autotune.h"
#include "bsp_IIC.h"
#include "bsp_AT24C02.h"
#include "bsp_tm1652.h"
#include "bsp_hlw8032.h"
#include "bsp_io.h"
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
void delay_us(uint16_t us);
//float Temp_Filter(float newVal);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI_SCK_Pin GPIO_PIN_0
#define SPI_SCK_GPIO_Port GPIOA
#define SPI_MISO_Pin GPIO_PIN_3
#define SPI_MISO_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_5
#define SPI_CS_GPIO_Port GPIOA
#define SDA_Pin GPIO_PIN_10
#define SDA_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_11
#define SCL_GPIO_Port GPIOB
#define Heat_pwm_Pin GPIO_PIN_8
#define Heat_pwm_GPIO_Port GPIOA
#define PV_SEG_Pin GPIO_PIN_4
#define PV_SEG_GPIO_Port GPIOB
#define SV_SEG_Pin GPIO_PIN_5
#define SV_SEG_GPIO_Port GPIOB
#define POW_SEG_Pin GPIO_PIN_6
#define POW_SEG_GPIO_Port GPIOB
#define VOL_SEG_Pin GPIO_PIN_7
#define VOL_SEG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

 /* 备用参数*/
//#define PID_PERIOD_MS 800  // PID 控制周期  500-> 800
//#define TPC_PERIOD_MS     8000    // 时间比例控制周期 2秒  2000->4000->8000

#define PID_PERIOD_MS 500  // PID 控制周期  500
#define TPC_PERIOD_MS     2000    // 时间比例控制周期 2秒
#define TEMP_PERIOD_MS 200  // 每 200ms 采集一次
#define TPC_STEPS         (TPC_PERIOD_MS / PID_PERIOD_MS)  // 每周期控制步数

#define BEEP_PORT       GPIOA
#define BEEP_PIN        GPIO_PIN_15

#define  Heat_on HAL_GPIO_WritePin(Heat_pwm_GPIO_Port, Heat_pwm_Pin, GPIO_PIN_SET);
#define  Heat_off HAL_GPIO_WritePin(Heat_pwm_GPIO_Port, Heat_pwm_Pin, GPIO_PIN_RESET);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
