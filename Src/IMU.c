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
LIS3MDL_Axes_t CompassAxes;
LIS3MDL_Object_t CompassObj;
LIS3MDL_IO_t CompassIO;

/* Accel lis331dl*/
H3LIS331DL_Axes_t AccelAxes;
H3LIS331DL_Object_t AccelObj;
H3LIS331DL_IO_t AccelIO;

/* Gyro L3G4200D*/
A3G4250D_Axes_t GyroAxes;
A3G4250D_Object_t GyroObj;
A3G4250D_IO_t GyroIO;

/* Barometer LPS331*/
float pressure;
float temperature;
LPS33HW_Object_t PressObj;
LPS33HW_IO_t PressIO;

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
    CompassIO.Init = CompassInit;
    CompassIO.DeInit = CompassDeInit;
    CompassIO.BusType = LIS3MDL_I2C_BUS;
    CompassIO.Address = LIS3MDL_I2C_ADD_L;
    CompassIO.ReadReg = IMURead;
    CompassIO.WriteReg = IMUWrite;

    if (LIS3MDL_RegisterBusIO(&CompassObj, &CompassIO) != LIS3MDL_OK)
    {
        while (1)
        {
            //Error
            vTaskDelay(1000);
        }
    }

    /* Accel lis331dl*/
    AccelIO.Init = AccelInit;
    AccelIO.DeInit = AccelDeInit;
    AccelIO.BusType = H3LIS331DL_I2C_BUS;
    AccelIO.Address = H3LIS331DL_I2C_ADD_L;
    AccelIO.ReadReg = IMURead;
    AccelIO.WriteReg = IMUWrite;

    if (H3LIS331DL_RegisterBusIO(&AccelObj, &AccelIO) != H3LIS331DL_OK)
    {
        while (1)
        {
            //Error
            vTaskDelay(1000);
        }
    }

    /* Gyro L3G4200D*/
    GyroIO.Init = GyroInit;
    GyroIO.DeInit = GyroDeInit;
    GyroIO.BusType = A3G4250D_I2C_BUS;
    GyroIO.Address = A3G4250D_I2C_ADD_L;
    GyroIO.ReadReg = IMURead;
    GyroIO.WriteReg = IMUWrite;

    if (A3G4250D_RegisterBusIO(&GyroObj, &GyroIO) != A3G4250D_OK)
    {
        while (1)
        {
            //Error
            vTaskDelay(1000);
        }
    }

    /* Barometer LPS331*/
    PressIO.Init = PressInit;
    PressIO.DeInit = PressDeInit;
    PressIO.BusType = LPS33HW_I2C_BUS;
    PressIO.Address = LPS33HW_I2C_ADD_L;
    PressIO.ReadReg = IMURead;
    PressIO.WriteReg = IMUWrite;

    if (LPS33HW_RegisterBusIO(&PressObj, &PressIO) != LPS33HW_OK)
    {
        while (1)
        {
            //Error
            vTaskDelay(1000);
        }
    }
    /* Infinite loop */

    for (;;)
    {
        vTaskDelay(1);
        LIS3MDL_MAG_GetAxes(&CompassObj, &CompassAxes);
        H3LIS331DL_ACC_GetAxes(&AccelObj, &AccelAxes);
        A3G4250D_GYRO_GetAxes(&GyroObj,&GyroAxes);
        LPS33HW_PRESS_GetPressure(&PressObj,&pressure);
        LPS33HW_TEMP_GetTemperature(&PressObj,&temperature);
    }
}

/*
*
* Read-write IMU
*******************************************************************************************
*/
int32_t IMUWrite(uint16_t adr, uint16_t RegAdr, uint8_t *data, uint16_t len)
{
    if (HAL_I2C_Mem_Write(&hi2c4, adr, RegAdr, sizeof(uint8_t), data, len, 500) == HAL_OK)
    {
        return 0;
    }
    return -1;
}

int32_t IMURead(uint16_t adr, uint16_t RegAdr, uint8_t *data, uint16_t len)
{
    if (HAL_I2C_Mem_Read(&hi2c4, adr, RegAdr, sizeof(uint8_t), data, len, 500) == HAL_OK)
    {
        return 0;
    }
    return -1;
}
/*
*
* Compass init
*******************************************************************************************
*/
int32_t CompassInit(void)
{
    if (LIS3MDL_Init(&CompassObj) != LIS3MDL_OK)
    {
        return LIS3MDL_ERROR;
    }
    if (LIS3MDL_MAG_SetFullScale(&CompassObj, LIS3MDL_4_GAUSS) != LIS3MDL_OK)
    {
        return LIS3MDL_ERROR;
    }
    if (LIS3MDL_MAG_SetOutputDataRate(&CompassObj, 80.0f) != LIS3MDL_OK)
    {
        return LIS3MDL_ERROR;
    }

    if (LIS3MDL_MAG_Enable(&CompassObj) != LIS3MDL_OK)
    {
        return LIS3MDL_ERROR;
    }

    return LIS3MDL_OK;
}

int32_t CompassDeInit(void)
{
    return LIS3MDL_DeInit(&CompassObj);
}

/*
*
* Accel init
*******************************************************************************************
*/
int32_t AccelInit(void)
{
    if (H3LIS331DL_Init(&AccelObj) != H3LIS331DL_OK)
    {
        return H3LIS331DL_ERROR;
    }
    if (H3LIS331DL_ACC_SetFullScale(&AccelObj, H3LIS331DL_ACC_SENSITIVITY_FOR_FS_200G) != H3LIS331DL_OK)
    {
        return H3LIS331DL_ERROR;
    }
    if (H3LIS331DL_ACC_SetOutputDataRate(&AccelObj, 100.0f) != H3LIS331DL_OK)
    {
        return H3LIS331DL_ERROR;
    }

    if (H3LIS331DL_ACC_Enable(&AccelObj) != H3LIS331DL_OK)
    {
        return H3LIS331DL_ERROR;
    }

    return H3LIS331DL_OK;
}

int32_t AccelDeInit(void)
{
    return LIS3MDL_DeInit(&CompassObj);
}

/*
*
* Gyro init
*******************************************************************************************
*/
int32_t GyroInit(void)
{
    if (A3G4250D_Init(&GyroObj) != A3G4250D_OK)
    {
        return A3G4250D_ERROR;
    }

    if (A3G4250D_GYRO_SetOutputDataRate(&GyroObj, 100.0f) != A3G4250D_OK)
    {
        return A3G4250D_ERROR;
    }

    if (A3G4250D_GYRO_Enable(&GyroObj) != A3G4250D_OK)
    {
        return A3G4250D_ERROR;
    }

    return A3G4250D_OK;
}

int32_t GyroDeInit(void)
{
    return A3G4250D_DeInit(&GyroObj);
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