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
#include "stm32f7xx_hal_flash.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

//#include <arm_math.h>

#include "a3g4250d.h"
#include "lis3mdl.h"
#include "lps331.h"
#include "h3lis331dl.h"
#include "stm32f7xx_nucleo_bus.h"
#include "custom_mems_conf.h"
#include "motion_fx.h"
#include "motion_mc.h"
#include "motion_vc.h"

#include "telemetry.h"
/* Private includes ----------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/
#define TimeSendTelemetry (uint16_t)10
#define FlashStartAdress ((uint32_t)0x081C0000) /* Base address of Sector 11, 256 Kbytes */
/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
int32_t PressInit(void);
int32_t PressDeInit(void);

/* Private defines -----------------------------------------------------------*/
#define LPS331_I2C_ADD_L   0xB9U


#ifdef __cplusplus
}
#endif

#endif /* __IMU_H */

/************************ Clear Sky LLC *****END OF FILE****/
