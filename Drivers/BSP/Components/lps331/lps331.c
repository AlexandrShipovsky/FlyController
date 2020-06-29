/**
 ******************************************************************************
 * @file    lps33hw.c
 * @author  MEMS Software Solutions Team
 * @brief   LPS33HW driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "lps331.h"

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup Component Component
 * @{
 */

/** @defgroup LPS33HW LPS33HW
 * @{
 */

/** @defgroup LPS33HW_Exported_Variables LPS33HW Exported Variables
 * @{
 */

LPS33HW_CommonDrv_t LPS33HW_COMMON_Driver =
    {
        LPS33HW_Init,
        LPS33HW_DeInit,
        LPS33HW_ReadID,
        LPS33HW_GetCapabilities,
};

LPS33HW_PRESS_Drv_t LPS33HW_PRESS_Driver =
    {
        LPS33HW_PRESS_Enable,
        LPS33HW_PRESS_Disable,
        LPS33HW_PRESS_GetOutputDataRate,
        LPS33HW_PRESS_SetOutputDataRate,
        LPS33HW_PRESS_GetPressure,
};

LPS33HW_TEMP_Drv_t LPS33HW_TEMP_Driver =
    {
        LPS33HW_TEMP_Enable,
        LPS33HW_TEMP_Disable,
        LPS33HW_TEMP_GetOutputDataRate,
        LPS33HW_TEMP_SetOutputDataRate,
        LPS33HW_TEMP_GetTemperature,
};

/**
 * @}
 */

/** @defgroup LPS33HW_Private_Function_Prototypes LPS33HW Private Function Prototypes
 * @{
 */

static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t LPS33HW_GetOutputDataRate(LPS33HW_Object_t *pObj, float *Odr);
static int32_t LPS33HW_SetOutputDataRate_When_Enabled(LPS33HW_Object_t *pObj, float Odr);
static int32_t LPS33HW_SetOutputDataRate_When_Disabled(LPS33HW_Object_t *pObj, float Odr);
static int32_t LPS33HW_Initialize(LPS33HW_Object_t *pObj);

/**
 * @}
 */

/** @defgroup LPS33HW_Exported_Functions LPS33HW Exported Functions
 * @{
 */

/**
 * @brief  Register Component Bus IO operations
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_RegisterBusIO(LPS33HW_Object_t *pObj, LPS33HW_IO_t *pIO)
{
  int32_t ret = LPS33HW_OK;

  if (pObj == NULL)
  {
    ret = LPS33HW_ERROR;
  }
  else
  {
    pObj->IO.Init = pIO->Init;
    pObj->IO.DeInit = pIO->DeInit;
    pObj->IO.BusType = pIO->BusType;
    pObj->IO.Address = pIO->Address;
    pObj->IO.WriteReg = pIO->WriteReg;
    pObj->IO.ReadReg = pIO->ReadReg;
    pObj->IO.GetTick = pIO->GetTick;

    pObj->Ctx.read_reg = ReadRegWrap;
    pObj->Ctx.write_reg = WriteRegWrap;
    pObj->Ctx.handle = pObj;

    if (pObj->IO.Init == NULL)
    {
      ret = LPS33HW_ERROR;
    }
    else if (pObj->IO.Init() != LPS33HW_OK)
    {
      ret = LPS33HW_ERROR;
    }
    else
    {
      if (pObj->IO.BusType == LPS33HW_SPI_3WIRES_BUS) /* SPI 3-Wires */
      {
        /* Enable the SPI 3-Wires support only the first time */
        if (pObj->is_initialized == 0U)
        {
          /* Enable SPI 3-Wires on the component */
          uint8_t data = 0x01;

          if (LPS33HW_Write_Reg(pObj, LPS33HW_CTRL_REG1, data) != LPS33HW_OK)
          {
            ret = LPS33HW_ERROR;
          }
        }
      }
    }
  }

  return ret;
}

/**
 * @brief  Initialize the LPS33HW sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_Init(LPS33HW_Object_t *pObj)
{
  if (pObj->is_initialized == 0U)
  {
    if (LPS33HW_Initialize(pObj) != LPS33HW_OK)
    {
      return LPS33HW_ERROR;
    }
  }

  pObj->is_initialized = 1U;

  return LPS33HW_OK;
}

/**
 * @brief  Deinitialize the LPS33HW sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_DeInit(LPS33HW_Object_t *pObj)
{
  if (pObj->is_initialized == 1U)
  {
    if (LPS33HW_PRESS_Disable(pObj) != LPS33HW_OK)
    {
      return LPS33HW_ERROR;
    }

    if (LPS33HW_TEMP_Disable(pObj) != LPS33HW_OK)
    {
      return LPS33HW_ERROR;
    }
  }

  pObj->is_initialized = 0;

  return LPS33HW_OK;
}

/**
 * @brief  Get WHO_AM_I value
 * @param  pObj the device pObj
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_ReadID(LPS33HW_Object_t *pObj, uint8_t *Id)
{
  if (lps33hw_device_id_get(&(pObj->Ctx), Id) != LPS33HW_OK)
  {
    return LPS33HW_ERROR;
  }

  return LPS33HW_OK;
}

/**
 * @brief  Get LPS33HW sensor capabilities
 * @param  pObj Component object pointer
 * @param  Capabilities pointer to LPS33HW sensor capabilities
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_GetCapabilities(LPS33HW_Object_t *pObj, LPS33HW_Capabilities_t *Capabilities)
{
  /* Prevent unused argument(s) compilation warning */
  (void)(pObj);

  Capabilities->Humidity = 0;
  Capabilities->Pressure = 1;
  Capabilities->Temperature = 1;
  Capabilities->LowPower = 0;
  Capabilities->HumMaxOdr = 0.0f;
  Capabilities->TempMaxOdr = 75.0f;
  Capabilities->PressMaxOdr = 75.0f;
  return LPS33HW_OK;
}

/**
 * @brief  Get the LPS33HW initialization status
 * @param  pObj the device pObj
 * @param  Status 1 if initialized, 0 otherwise
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_Get_Init_Status(LPS33HW_Object_t *pObj, uint8_t *Status)
{
  if (pObj == NULL)
  {
    return LPS33HW_ERROR;
  }

  *Status = pObj->is_initialized;

  return LPS33HW_OK;
}

/**
 * @brief  Enable the LPS33HW pressure sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_PRESS_Enable(LPS33HW_Object_t *pObj)
{
  /* Check if the component is already enabled */
  if (pObj->press_is_enabled == 1U)
  {
    return LPS33HW_OK;
  }

  /* Output data rate selection. */
  if (lps33hw_data_rate_set(&(pObj->Ctx), pObj->last_odr) != LPS33HW_OK)
  {
    return LPS33HW_ERROR;
  }

  /* Power up selection. */
  if (lps33hw_power_set(&(pObj->Ctx), PROPERTY_ENABLE) != LPS33HW_OK)
  {
    return LPS33HW_ERROR;
  }

  pObj->press_is_enabled = 1;

  return LPS33HW_OK;
}

/**
 * @brief  Disable the LPS33HW pressure sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_PRESS_Disable(LPS33HW_Object_t *pObj)
{
  /* Check if the component is already disabled */
  if (pObj->press_is_enabled == 0U)
  {
    return LPS33HW_OK;
  }

  /* Check if the LPS33HW temperature sensor is still enable. */
  /* If yes, skip the disable function, if not call disable function */
  if (pObj->temp_is_enabled == 0U)
  {
    /* Get current output data rate. */
    if (lps33hw_data_rate_get(&(pObj->Ctx), &pObj->last_odr) != LPS33HW_OK)
    {
      return LPS33HW_ERROR;
    }

    /* Output data rate selection - power down. */
    if (lps33hw_data_rate_set(&(pObj->Ctx), LPS33HW_POWER_DOWN) != LPS33HW_OK)
    {
      return LPS33HW_ERROR;
    }
  }

  pObj->press_is_enabled = 0;

  return LPS33HW_OK;
}

/**
 * @brief  Get the LPS33HW pressure sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_PRESS_GetOutputDataRate(LPS33HW_Object_t *pObj, float *Odr)
{
  return LPS33HW_GetOutputDataRate(pObj, Odr);
}

/**
 * @brief  Set the LPS33HW pressure sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_PRESS_SetOutputDataRate(LPS33HW_Object_t *pObj, float Odr)
{
  /* Check if the component is enabled */
  if (pObj->press_is_enabled == 1U)
  {
    return LPS33HW_SetOutputDataRate_When_Enabled(pObj, Odr);
  }
  else
  {
    return LPS33HW_SetOutputDataRate_When_Disabled(pObj, Odr);
  }
}

/**
 * @brief  Get the LPS33HW pressure value
 * @param  pObj the device pObj
 * @param  Value pointer where the pressure value is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_PRESS_GetPressure(LPS33HW_Object_t *pObj, float *Value)
{
  //lps33hw_axis1bit32_t data_raw_pressure;
  //(void)memset(data_raw_pressure.u8bit, 0x00, sizeof(int32_t));
  uint8_t buf[6];
  memset(buf, 0x00, sizeof(buf));
  if (lps33hw_read_reg(&(pObj->Ctx), LPS33HW_PRESS_OUT_XL, buf, sizeof(buf)) != LPS33HW_OK)
  {
    return LPS33HW_ERROR;
  }

  *Value = ((int32_t)buf[2] << 16 | (uint16_t)buf[1] << 8 | buf[0]) / 4096.0;

  return LPS33HW_OK;
}

/**
 * @brief  Enable the LPS33HW temperature sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_TEMP_Enable(LPS33HW_Object_t *pObj)
{
  /* Check if the component is already enabled */
  if (pObj->temp_is_enabled == 1U)
  {
    return LPS33HW_OK;
  }

  /* Output data rate selection. */
  if (lps33hw_data_rate_set(&(pObj->Ctx), pObj->last_odr) != LPS33HW_OK)
  {
    return LPS33HW_ERROR;
  }

  pObj->temp_is_enabled = 1;

  return LPS33HW_OK;
}

/**
 * @brief  Disable the LPS33HW temperature sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_TEMP_Disable(LPS33HW_Object_t *pObj)
{
  /* Check if the component is already disabled */
  if (pObj->temp_is_enabled == 0U)
  {
    return LPS33HW_OK;
  }

  /* Check if the LPS33HW pressure sensor is still enable. */
  /* If yes, skip the disable function, if not call disable function */
  if (pObj->press_is_enabled == 0U)
  {
    /* Get current output data rate. */
    if (lps33hw_data_rate_get(&(pObj->Ctx), &pObj->last_odr) != LPS33HW_OK)
    {
      return LPS33HW_ERROR;
    }

    /* Output data rate selection - power down. */
    if (lps33hw_data_rate_set(&(pObj->Ctx), LPS33HW_POWER_DOWN) != LPS33HW_OK)
    {
      return LPS33HW_ERROR;
    }
  }

  pObj->temp_is_enabled = 0;

  return LPS33HW_OK;
}

/**
 * @brief  Get the LPS33HW temperature sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_TEMP_GetOutputDataRate(LPS33HW_Object_t *pObj, float *Odr)
{
  return LPS33HW_GetOutputDataRate(pObj, Odr);
}

/**
 * @brief  Set the LPS33HW temperature sensor output data rate
 * @param  pObj the device pObj
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_TEMP_SetOutputDataRate(LPS33HW_Object_t *pObj, float Odr)
{
  /* Check if the component is enabled */
  if (pObj->temp_is_enabled == 1U)
  {
    return LPS33HW_SetOutputDataRate_When_Enabled(pObj, Odr);
  }
  else
  {
    return LPS33HW_SetOutputDataRate_When_Disabled(pObj, Odr);
  }
}

/**
 * @brief  Get the LPS33HW temperature value
 * @param  pObj the device pObj
 * @param  Value pointer where the temperature value is written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_TEMP_GetTemperature(LPS33HW_Object_t *pObj, float *Value)
{
  uint8_t buf[2];

  memset(buf, 0x00, sizeof(buf));
  if (lps33hw_read_reg(&(pObj->Ctx), LPS33HW_TEMP_OUT_L, buf, sizeof(buf)) != LPS33HW_OK)
  {
    return LPS33HW_ERROR;
  }
  int16_t temp = ((uint16_t)buf[1] << 8 | buf[0]);
  *Value = 42.5 + temp / 480.0;

  return LPS33HW_OK;
}

/**
 * @brief  Get the LPS33HW temperature value
 * @param  pObj the device pObj
 * @param  Value pointer where the temperature value is written
 * @retval 0 in case of success, an error code otherwise
 */
void LPS331_Get_Altitude(LPS33HW_Object_t *pObj, float TempZero, float PressZero, float *Data)
{
  float pressure, temperature;

  LPS33HW_PRESS_GetPressure(pObj, &pressure);
  LPS33HW_TEMP_GetTemperature(pObj, &temperature);

  *Data = 18400*(1 + 0.003665*25)*log10(PressZero/pressure);
}
/**
 * @brief  Get the LPS33HW register value
 * @param  pObj the device pObj
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_Read_Reg(LPS33HW_Object_t *pObj, uint8_t Reg, uint8_t *Data)
{
  if (lps33hw_read_reg(&(pObj->Ctx), Reg, Data, 1) != LPS33HW_OK)
  {
    return LPS33HW_ERROR;
  }

  return LPS33HW_OK;
}

/**
 * @brief  Set the LPS33HW register value
 * @param  pObj the device pObj
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_Write_Reg(LPS33HW_Object_t *pObj, uint8_t Reg, uint8_t Data)
{
  if (lps33hw_write_reg(&(pObj->Ctx), Reg, &Data, 1) != LPS33HW_OK)
  {
    return LPS33HW_ERROR;
  }

  return LPS33HW_OK;
}

/**
 * @}
 */

/** @defgroup LPS33HW_Private_Functions LPS33HW Private Functions
 * @{
 */

/**
 * @brief  Get output data rate
 * @param  pObj the device pObj
 * @param  Odr the output data rate value
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t LPS33HW_GetOutputDataRate(LPS33HW_Object_t *pObj, float *Odr)
{
  int32_t ret = LPS33HW_OK;
  lps33hw_odr_t odr_low_level;

  if (lps33hw_data_rate_get(&(pObj->Ctx), &odr_low_level) != LPS33HW_OK)
  {
    return LPS33HW_ERROR;
  }

  switch (odr_low_level)
  {
  case LPS33HW_POWER_DOWN:
    *Odr = 0.0f;
    break;

  case LPS33HW_ODR_P1_Hz_T1_Hz:
    *Odr = 1.0f;
    break;

  case LPS33HW_ODR_P7_Hz_T1_Hz:
    *Odr = 7.0f;
    break;

  case LPS33HW_ODR_P12_Hz_T1_Hz:
    *Odr = 12.5f;
    break;

  case LPS33HW_ODR_P25_Hz_T1_Hz:
    *Odr = 25.0f;
    break;

  case LPS33HW_ODR_P25_Hz_T25_Hz:
    *Odr = 50.0f;
    break;

  default:
    ret = LPS33HW_ERROR;
    break;
  }

  return ret;
}

/**
 * @brief  Set output data rate
 * @param  pObj the device pObj
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t LPS33HW_SetOutputDataRate_When_Enabled(LPS33HW_Object_t *pObj, float Odr)
{
  lps33hw_odr_t new_odr;

  new_odr = (Odr <= 1.0f) ? LPS33HW_ODR_P1_Hz_T1_Hz
                          : (Odr <= 7.0f) ? LPS33HW_ODR_P7_Hz_T1_Hz
                                          : (Odr <= 12.5f) ? LPS33HW_ODR_P12_Hz_T1_Hz
                                                           : (Odr <= 25.0f) ? LPS33HW_ODR_P25_Hz_T1_Hz
                                                                            : LPS33HW_ODR_P7_Hz_T7_Hz;

  if (lps33hw_data_rate_set(&(pObj->Ctx), new_odr) != LPS33HW_OK)
  {
    return LPS33HW_ERROR;
  }

  if (lps33hw_data_rate_get(&(pObj->Ctx), &pObj->last_odr) != LPS33HW_OK)
  {
    return LPS33HW_ERROR;
  }

  return LPS33HW_OK;
}

/**
 * @brief  Set output data rate when disabled
 * @param  pObj the device pObj
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t LPS33HW_SetOutputDataRate_When_Disabled(LPS33HW_Object_t *pObj, float Odr)
{
  pObj->last_odr = (Odr <= 1.0f) ? LPS33HW_ODR_P1_Hz_T1_Hz
                                 : (Odr <= 7.0f) ? LPS33HW_ODR_P7_Hz_T1_Hz
                                                 : (Odr <= 12.5f) ? LPS33HW_ODR_P12_Hz_T1_Hz
                                                                  : (Odr <= 25.0f) ? LPS33HW_ODR_P25_Hz_T1_Hz
                                                                                   : LPS33HW_ODR_P7_Hz_T7_Hz;

  return LPS33HW_OK;
}

/**
 * @brief  Initialize the LPS33HW sensor
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t LPS33HW_Initialize(LPS33HW_Object_t *pObj)
{
  /* Power down the device */
  if (lps33hw_data_rate_set(&(pObj->Ctx), LPS33HW_POWER_DOWN) != LPS33HW_OK)
  {
    return LPS33HW_ERROR;
  }

  /* Set block data update mode */
  if (lps33hw_block_data_update_set(&(pObj->Ctx), PROPERTY_ENABLE) != LPS33HW_OK)
  {
    return LPS33HW_ERROR;
  }

  pObj->last_odr = LPS33HW_ODR_P7_Hz_T7_Hz;

  return LPS33HW_OK;
}

/**
 * @brief  Set the LPS33HW One Shot Mode
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_Set_One_Shot(LPS33HW_Object_t *pObj)
{
  /* Set ODR */
  if (lps33hw_data_rate_set(&(pObj->Ctx), LPS33HW_POWER_DOWN) != LPS33HW_OK)
  {
    return LPS33HW_ERROR;
  }

  /* Start One Shot Measurement */
  if (lps33hw_one_shoot_trigger_set(&(pObj->Ctx), PROPERTY_ENABLE) != LPS33HW_OK)
  {
    return LPS33HW_ERROR;
  }

  return LPS33HW_OK;
}

/**
 * @brief  Get the LPS33HW One Shot Status
 * @param  pObj the device pObj
 * @param  Status pointer to the one shot status (1 means measurements available, 0 means measurements not available yet)
 * @retval 0 in case of success, an error code otherwise
 */
int32_t LPS33HW_Get_One_Shot_Status(LPS33HW_Object_t *pObj, uint8_t *Status)
{
  uint8_t p_da;
  uint8_t t_da;

  /* Get DataReady for pressure */
  if (lps33hw_press_data_ready_get(&(pObj->Ctx), &p_da) != LPS33HW_OK)
  {
    return LPS33HW_ERROR;
  }

  /* Get DataReady for temperature */
  if (lps33hw_temp_data_ready_get(&(pObj->Ctx), &t_da) != LPS33HW_OK)
  {
    return LPS33HW_ERROR;
  }

  if (p_da && t_da)
  {
    *Status = 1;
  }
  else
  {
    *Status = 0;
  }

  return LPS33HW_OK;
}

/**
 * @brief  Wrap Read register component function to Bus IO function
 * @param  Handle the device handler
 * @param  Reg the register address
 * @param  pData the stored data pointer
 * @param  Length the length
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  LPS33HW_Object_t *pObj = (LPS33HW_Object_t *)Handle;

  return pObj->IO.ReadReg(pObj->IO.Address, (0x80 | Reg), pData, Length);
}

/**
 * @brief  Wrap Write register component function to Bus IO function
 * @param  Handle the device handler
 * @param  Reg the register address
 * @param  pData the stored data pointer
 * @param  Length the length
 * @retval 0 in case of success, an error code otherwise
 */
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  LPS33HW_Object_t *pObj = (LPS33HW_Object_t *)Handle;

  return pObj->IO.WriteReg(pObj->IO.Address, (0x80 | Reg), pData, Length);
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
