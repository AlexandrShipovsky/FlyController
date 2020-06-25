/**
  ******************************************************************************
  * File Name          : IMU.c
  * Description        : 
  ******************************************************************************
  * @attention
  *
  *
  *
  ******************************************************************************
  */
/* Private define ------------------------------------------------------------*/
#include "IMU.h"

/* Private variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c4;
LIS3MDL_Object_t lis3mdl_obj;
LIS3MDL_IO_t lis3mdl_IO;

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void const *argument)
{

    lis3mdl_IO.Init = CompassInit;
    lis3mdl_IO.DeInit = CompassDeInit;
    lis3mdl_IO.BusType = LIS3MDL_I2C_BUS;
    /* Infinite loop */

    for (;;)
    {
        vTaskDelay(1);
    }
}

int32_t CompassInit(void)
{
    return LIS3MDL_OK;
}

int32_t CompassDeInit(void)
{
    return LIS3MDL_OK;
}

int32_t CompassWrite(uint16_t adr, uint16_t RegAdr, uint8_t *data, uint16_t lenth)
{
    return LIS3MDL_OK;
}

int32_t CompassRead(uint16_t adr, uint16_t RegAdr, uint8_t *data, uint16_t lenth)
{
    return LIS3MDL_OK;
}