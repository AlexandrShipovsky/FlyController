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
float CompassAxesCalib[3];
float CompassAxesRawmT[3];
LIS3MDL_Object_t CompassObj;
LIS3MDL_IO_t CompassIO;

MMC_Input_t MagCalibIn;
MMC_Output_t MagCalibOut;

/* Accel lis331dl*/
H3LIS331DL_Axes_t AccelAxes;
H3LIS331DL_Object_t AccelObj;
H3LIS331DL_IO_t AccelIO;

/* Gyro L3G4200D*/
A3G4250D_Axes_t GyroAxes;
A3G4250D_Object_t GyroObj;
A3G4250D_IO_t GyroIO;

/* Barometer LPS331*/
float PressureZero;
float TemperatureZero;
float AltitudeZero = 0.0;

float pressure;
float temperature;
float altitude;
LPS33HW_Object_t PressObj;
LPS33HW_IO_t PressIO;

MVC_input_t VerticalConextIn;
MVC_output_t OutMotionVC;

MFX_knobs_t KnobsMFX;
MFX_input_t InMFX;
MFX_output_t OutMFX;

SemaphoreHandle_t SemaphoreForSendTelemetry; // Семафор, разрешающий отправку телеметрии с ИНС раз в TimeSendTelemetry*vTaskDelay миллисекунд
IMUTelemetryTypeDef IMUTelemetry;
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

    MotionVC_Initialize();

    /* Получение значения высоты над уровнем моря в начальной точке*/
    while (AltitudeZero == 0.0)
    {
        H3LIS331DL_ACC_GetAxes(&AccelObj, &AccelAxes);
        LPS33HW_PRESS_GetPressure(&PressObj, &pressure);
        vTaskDelay(100);
        VerticalConextIn.AccX = AccelAxes.x / 1000.0;
        VerticalConextIn.AccY = AccelAxes.y / 1000.0;
        VerticalConextIn.AccZ = AccelAxes.z / 1000.0;
        VerticalConextIn.Press = pressure;

        MotionVC_Update(&VerticalConextIn, &OutMotionVC);

        AltitudeZero = OutMotionVC.Baro_Altitude / 100.0;
    }

    /* Калибровка магнитометра*/
    /*************************************************************************************************/
    /*************************************************************************************************/
    MotionMC_Initialize(25, 1);
    MotionMC_GetCalParams(&MagCalibOut);

    while (MagCalibOut.CalQuality != MMC_CALQSTATUSGOOD)
    {
        LIS3MDL_MAG_GetAxes(&CompassObj, &CompassAxes);
        MagCalibIn.Mag[0] = CompassAxes.x / 10.0;
        MagCalibIn.Mag[1] = CompassAxes.y / 10.0;
        MagCalibIn.Mag[2] = CompassAxes.z / 10.0;
        MagCalibIn.TimeStamp = osKernelSysTick();
        MotionMC_Update(&MagCalibIn);
        MotionMC_GetCalParams(&MagCalibOut);
        vTaskDelay(25);
    }
    MotionMC_Initialize(25, 0);
    /* MotionFX run*/
    MotionFX_initialize();
    /*************************************************************************************************/
    /*************************************************************************************************/
    MotionFX_getKnobs(&KnobsMFX);
    KnobsMFX.LMode = 2; // Динамическая коррекция гироскопа
    KnobsMFX.modx = 5;  // Встраиваемая система
    KnobsMFX.FrTime = 10;
    KnobsMFX.output_type = MFX_ENGINE_OUTPUT_ENU;
    MotionFX_setKnobs(&KnobsMFX);
    MotionFX_enable_9X(MFX_ENGINE_ENABLE);
    MotionFX_enable_6X(MFX_ENGINE_ENABLE);

    uint32_t tick_current = 0;
    uint32_t tick_prev = 0;
    float deltaTimeMFX;

    uint16_t IncForSendTelemetry = 0;
    SemaphoreForSendTelemetry = xSemaphoreCreateBinary();
    /* Infinite loop */
    for (;;)
    {
        vTaskDelay(10);
        IncForSendTelemetry++;

        /* Высота*/
        H3LIS331DL_ACC_GetAxes(&AccelObj, &AccelAxes);
        LPS33HW_PRESS_GetPressure(&PressObj, &pressure);

        VerticalConextIn.AccX = (float)AccelAxes.x / 1000.0;
        VerticalConextIn.AccY = (float)AccelAxes.y / 1000.0;
        VerticalConextIn.AccZ = (float)AccelAxes.z / 1000.0;
        VerticalConextIn.Press = pressure;

        MotionVC_Update(&VerticalConextIn, &OutMotionVC);

        altitude = OutMotionVC.Baro_Altitude / 100.0 - AltitudeZero;

        /* Расчет значений компаса*/
        MotionMC_GetCalParams(&MagCalibOut);
        LIS3MDL_MAG_GetAxes(&CompassObj, &CompassAxes);
        CompassAxesRawmT[0] = (float)CompassAxes.x / 10.0;
        CompassAxesRawmT[1] = (float)CompassAxes.y / 10.0;
        CompassAxesRawmT[2] = (float)CompassAxes.z / 10.0;

        for (uint8_t i = 0; i < 3; i++)
        {
            CompassAxesCalib[i] = 0.0;
            for (uint8_t j = 0; j < 3; j++)
            {
                CompassAxesCalib[i] += MagCalibOut.SF_Matrix[i][j] * (CompassAxesRawmT[i] - MagCalibOut.HI_Bias[i]);
            }
        }

        A3G4250D_GYRO_GetAxes(&GyroObj, &GyroAxes);

        for (uint8_t i = 0; i < 3; i++)
        {
            InMFX.mag[i] = (float)CompassAxesCalib[i] / 50.0; // uT/50
        }

        InMFX.acc[0] = (float)AccelAxes.x / 1000.0;
        InMFX.acc[1] = (float)AccelAxes.y / 1000.0;
        InMFX.acc[2] = (float)AccelAxes.z / 1000.0;

        InMFX.gyro[0] = (float)GyroAxes.x / 1000.0;
        InMFX.gyro[1] = (float)GyroAxes.y / 1000.0;
        InMFX.gyro[2] = (float)GyroAxes.z / 1000.0;

        tick_current = osKernelSysTick();
        deltaTimeMFX = (tick_current - tick_prev) / 1000.0;
        MotionFX_propagate(&OutMFX, &InMFX, &deltaTimeMFX);
        MotionFX_update(&OutMFX, &InMFX, &deltaTimeMFX, NULL);
        tick_prev = tick_current;

        LPS33HW_TEMP_GetTemperature(&PressObj, &temperature);

        if (IncForSendTelemetry > TimeSendTelemetry)
        {
            if (SemaphoreForSendTelemetry != NULL)
            {
                if (xSemaphoreGive(SemaphoreForSendTelemetry) != pdTRUE)
                {
                }
                if (xSemaphoreTake(SemaphoreForSendTelemetry, (TickType_t)0))
                {
                    IMUTelemetry.altitude = altitude;
                    IMUTelemetry.yaw = OutMFX.rotation_9X[0];
                    IMUTelemetry.pitch = OutMFX.rotation_9X[1];
                    IMUTelemetry.roll = OutMFX.rotation_9X[2];
                     if (xSemaphoreGive(SemaphoreForSendTelemetry) != pdTRUE)
                    {
                    }
                }
            }
            IncForSendTelemetry = 0;
        }
        //LPS331_Get_Altitude(&PressObj,TemperatureZero,PressureZero,&AverageBuf[i]);
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

char MotionMC_LoadCalFromNVM(unsigned short int datasize, unsigned int *data)
{
    return 1; /* FAILURE: Read from NVM not implemented. */
}
char MotionMC_SaveCalInNVM(unsigned short int datasize, unsigned int *data)
{
    return 1; /* FAILURE: Write to NVM not implemented. */
}