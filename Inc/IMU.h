/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : IMU.h
  * @brief          : Header for IMU.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "a3g4250d.h"
#include "lis3mdl.h"
#include "lps33hw.h"
#include "h3lis331dl.h"

/* Private includes ----------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
int32_t CompassInit(void);
int32_t CompassDeInit(void);
int32_t CompassWrite(uint16_t adr, uint16_t RegAdr, uint8_t *data, uint16_t lenth);
int32_t CompassRead(uint16_t adr, uint16_t RegAdr, uint8_t *data, uint16_t lenth);

/* Private defines -----------------------------------------------------------*/



#ifdef __cplusplus
}
#endif

#endif /* __IMU_H */

/************************ Clear Sky LLC *****END OF FILE****/
