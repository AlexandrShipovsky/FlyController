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

/* Compass LIS3MDL*/
LIS3MDL_AxesRaw_t CompassAxesRaw;
LIS3MDL_Object_t CompassObj;
LIS3MDL_IO_t CompassIO;

/* Accel lis331dl*/
H3LIS331DL_AxesRaw_t AccelAxesRaw;
H3LIS331DL_Object_t AccelObj;
H3LIS331DL_IO_t AccelIO;

/* Gyro L3G4200D*/
A3G4250D_AxesRaw_t GyroAxesRaw;
A3G4250D_Object_t GyroObj;
A3G4250D_IO_t GyroIO;

/* Barometer LPS331*/
float PressureZero;
float TemperatureZero;
float altitudesum = 0.0;

float pressure;
float temperature;
float altitude;
LPS33HW_Object_t PressObj;
LPS33HW_IO_t PressIO;

uint8_t ID;
float rate;

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void const *argument)
{
    /* Compass LIS3MDL*/
    CompassIO.Init = CUSTOM_LIS3MDL_0_I2C_Init;
    CompassIO.DeInit = CUSTOM_LIS3MDL_0_I2C_DeInit;
    CompassIO.BusType = LIS3MDL_I2C_BUS;
    CompassIO.Address = LIS3MDL_I2C_ADD_L;
    CompassIO.ReadReg = CUSTOM_LIS3MDL_0_I2C_ReadReg;
    CompassIO.WriteReg = CUSTOM_LIS3MDL_0_I2C_WriteReg;

    if (LIS3MDL_RegisterBusIO(&CompassObj, &CompassIO) != LIS3MDL_OK)
    {
        while (1)
        {
            //Error
            vTaskDelay(1000);
        }
    }

    LIS3MDL_Init(&CompassObj);
    LIS3MDL_MAG_Enable(&CompassObj);

    /* Accel lis331dl*/
    AccelIO.Init = CUSTOM_H3LIS331DL_0_I2C_Init;
    AccelIO.DeInit = CUSTOM_H3LIS331DL_0_I2C_DeInit;
    AccelIO.BusType = H3LIS331DL_I2C_BUS;
    AccelIO.Address = H3LIS331DL_I2C_ADD_L;
    AccelIO.ReadReg = CUSTOM_H3LIS331DL_0_I2C_ReadReg;
    AccelIO.WriteReg = CUSTOM_H3LIS331DL_0_I2C_WriteReg;

    if (H3LIS331DL_RegisterBusIO(&AccelObj, &AccelIO) != H3LIS331DL_OK)
    {
        while (1)
        {
            //Error
            vTaskDelay(1000);
        }
    }

    H3LIS331DL_Init(&AccelObj);
    H3LIS331DL_ACC_Enable(&AccelObj);

    /* Gyro L3G4200D*/
    GyroIO.Init = CUSTOM_A3G4250D_0_I2C_Init;
    GyroIO.DeInit = CUSTOM_A3G4250D_0_I2C_DeInit;
    GyroIO.BusType = A3G4250D_I2C_BUS;
    GyroIO.Address = A3G4250D_I2C_ADD_L;
    GyroIO.ReadReg = CUSTOM_A3G4250D_0_I2C_ReadReg;
    GyroIO.WriteReg = CUSTOM_A3G4250D_0_I2C_WriteReg;

    if (A3G4250D_RegisterBusIO(&GyroObj, &GyroIO) != A3G4250D_OK)
    {
        while (1)
        {
            //Error
            vTaskDelay(1000);
        }
    }

    A3G4250D_Init(&GyroObj);
    A3G4250D_GYRO_Enable(&GyroObj);
    /* Barometer LPS331*/
    PressIO.Init = CUSTOM_LPS331_0_I2C_Init;
    PressIO.DeInit = CUSTOM_LPS331_0_I2C_DeInit;
    PressIO.BusType = LPS33HW_I2C_BUS;
    PressIO.Address = LPS331_I2C_ADD_L;
    PressIO.ReadReg = CUSTOM_LPS331_0_I2C_ReadReg;
    PressIO.WriteReg = CUSTOM_LPS331_0_I2C_WriteReg;

    if (LPS33HW_RegisterBusIO(&PressObj, &PressIO) != LPS33HW_OK)
    {
        while (1)
        {
            //Error
            vTaskDelay(1000);
        }
    }

    LPS33HW_Init(&PressObj);
    LPS33HW_PRESS_Enable(&PressObj);
    
    float AverageBuf[100];

    LPS33HW_PRESS_GetPressure(&PressObj,&AverageBuf[0]);
    vTaskDelay(500);

    uint8_t i = 0;
    for(i = 0; i < 20 ; i++)
    {
        LPS33HW_PRESS_GetPressure(&PressObj,&AverageBuf[i]);
        PressureZero += AverageBuf[i];
        vTaskDelay(50);
    }
    PressureZero /= 20.0;
    
    LPS33HW_TEMP_GetTemperature(&PressObj,&TemperatureZero);
    /* Infinite loop */
    i = 0;
    for (;;)
    {
        vTaskDelay(10);
        LIS3MDL_MAG_GetAxesRaw(&CompassObj, &CompassAxesRaw);
        H3LIS331DL_ACC_GetAxesRaw(&AccelObj, &AccelAxesRaw);
        A3G4250D_GYRO_GetAxesRaw(&GyroObj, &GyroAxesRaw);

        LPS33HW_ReadID(&PressObj,&ID);

        LPS33HW_PRESS_GetPressure(&PressObj,&pressure);
        
        LPS33HW_PRESS_GetOutputDataRate(&PressObj,&rate);
        LPS33HW_TEMP_GetTemperature(&PressObj,&temperature);

        LPS331_Get_Altitude(&PressObj,TemperatureZero,PressureZero,&AverageBuf[i]);
        altitudesum += AverageBuf[i];
        i++;
        if(i == 20)
        {
            altitude = altitudesum / 20.0;
            i = 0;
            altitudesum = 0.0;
        }
    }
}

/*
*
* Barometer init
*******************************************************************************************
*/
int32_t PressInit(void)
{
    if (LPS33HW_Init(&PressObj) != LPS33HW_OK)
    {
        return LPS33HW_ERROR;
    }

    if (LPS33HW_PRESS_SetOutputDataRate(&PressObj, 10.0f) != LPS33HW_OK)
    {
        return LPS33HW_ERROR;
    }

    if (LPS33HW_TEMP_SetOutputDataRate(&PressObj, 1.0f) != LPS33HW_OK)
    {
        return LPS33HW_ERROR;
    }

    if (LPS33HW_PRESS_Enable(&PressObj) != LPS33HW_OK)
    {
        return LPS33HW_ERROR;
    }
    if (LPS33HW_TEMP_Enable(&PressObj) != LPS33HW_OK)
    {
        return LPS33HW_ERROR;
    }

    return LPS33HW_OK;
}

int32_t PressDeInit(void)
{
    return LPS33HW_DeInit(&PressObj);
}