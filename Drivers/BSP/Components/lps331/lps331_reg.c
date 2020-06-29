/*
 ******************************************************************************
 * @file    lps33hw_reg.c
 * @author  Sensors Software Solution Team
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

#include "lps331_reg.h"

/**
  * @defgroup    LPS33HW
  * @brief       This file provides a set of functions needed to drive the
  *              ultra-compact piezoresistive absolute pressure sensor.
  * @{
  *
  */

/** 
  * @defgroup    LPS33HW_Interfaces_functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  * @{
  *
  */

/**
  * @brief  Read generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps33hw_read_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
                         uint16_t len)
{
  int32_t ret;
  ret = ctx->read_reg(ctx->handle, reg, data, len);
  return ret;
}

/**
  * @brief  Write generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lps33hw_write_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
                          uint16_t len)
{
  int32_t ret;
  ret = ctx->write_reg(ctx->handle, reg, data, len);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    LPS33HW_Sensitivity
  * @brief       These functions convert raw-data into engineering units.
  * @{
  *
  */

float_t lps33hw_from_lsb_to_hpa(int32_t lsb)
{
  return ( (float_t)lsb / 4096.0f );
}

float_t lps33hw_from_lsb_to_degc(int16_t lsb)
{
  return ( (float_t)lsb / 100.0f );
}

/**
  * @}
  *
  */

/**
  * @defgroup    LPS33HW_data_generation_c
  * @brief       This section group all the functions concerning data
  *              generation
  * @{
  *
  */


/**
  * @brief  Reset Autozero function.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of reset_az in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */

int32_t lps33hw_autozero_rst_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps33hw_interrupt_cfg_t interrupt_cfg;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_INTERRUPT_CFG,
                         (uint8_t*)&interrupt_cfg, 1);
  if(ret == 0){
    interrupt_cfg.reset_az = val;
    ret = lps33hw_write_reg(ctx, LPS33HW_INTERRUPT_CFG,
                            (uint8_t*)&interrupt_cfg, 1);
  }
  return ret;
}

/**
  * @brief  Reset Autozero function.[get] 
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of reset_az in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_autozero_rst_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33hw_interrupt_cfg_t interrupt_cfg;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_INTERRUPT_CFG,
                         (uint8_t*)&interrupt_cfg, 1);
  *val = interrupt_cfg.reset_az;

  return ret;
}

/**
  * @brief  Enable Autozero function.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of autozero in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_autozero_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps33hw_interrupt_cfg_t interrupt_cfg;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_INTERRUPT_CFG, 
                         (uint8_t*)&interrupt_cfg, 1);
  if(ret == 0){
    interrupt_cfg.autozero = val;
    ret = lps33hw_write_reg(ctx, LPS33HW_INTERRUPT_CFG,
                            (uint8_t*)&interrupt_cfg, 1);
  }
  return ret;
}

/**
  * @brief  Enable Autozero function.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of autozero in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_autozero_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33hw_interrupt_cfg_t interrupt_cfg;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_INTERRUPT_CFG,
                         (uint8_t*)&interrupt_cfg, 1);
  *val = interrupt_cfg.autozero;

  return ret;
}

/**
  * @brief  Reset AutoRifP function.[set] 
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of reset_arp in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_pressure_snap_rst_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps33hw_interrupt_cfg_t interrupt_cfg;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_INTERRUPT_CFG,
                         (uint8_t*)&interrupt_cfg, 1);
  if(ret == 0){
    interrupt_cfg.reset_arp = val;
    ret = lps33hw_write_reg(ctx, LPS33HW_INTERRUPT_CFG,
                            (uint8_t*)&interrupt_cfg, 1);
  }
  return ret;
}

/**
  * @brief  Reset AutoRifP function.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of reset_arp in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_pressure_snap_rst_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33hw_interrupt_cfg_t interrupt_cfg;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_INTERRUPT_CFG,
                         (uint8_t*)&interrupt_cfg, 1);
  *val = interrupt_cfg.reset_arp;

  return ret;
}

/**
  * @brief  Enable AutoRifP function.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of autorifp in reg INTERRUPT_CFG.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_pressure_snap_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps33hw_interrupt_cfg_t interrupt_cfg;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_INTERRUPT_CFG,
                         (uint8_t*)&interrupt_cfg, 1);
  if(ret == 0){
    interrupt_cfg.autorifp = val;
    ret = lps33hw_write_reg(ctx, LPS33HW_INTERRUPT_CFG,
                            (uint8_t*)&interrupt_cfg, 1);
  }
  return ret;
}

/**
  * @brief  Enable AutoRifP function.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of autorifp in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_pressure_snap_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33hw_interrupt_cfg_t interrupt_cfg;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_INTERRUPT_CFG,
                         (uint8_t*)&interrupt_cfg, 1);
  *val = interrupt_cfg.autorifp;

  return ret;
}

/**
  * @brief  Block data update.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of bdu in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps33hw_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  if(ret == 0){
    ctrl_reg1.dbdu = val;
    ret = lps33hw_write_reg(ctx, LPS33HW_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  }
  return ret;
}

/**
  * @brief  Block data update.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of bdu in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33hw_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  *val = ctrl_reg1.dbdu;

  return ret;
}



/**
  * @brief  Output data rate selection.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of odr in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_data_rate_set(stmdev_ctx_t *ctx, lps33hw_odr_t val)
{
  lps33hw_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  if(ret == 0){
    ctrl_reg1.odr = (uint8_t)val;
    ret = lps33hw_write_reg(ctx, LPS33HW_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  }
  return ret;
}

/**
  * @brief  Power up selection.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of odr in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_power_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps33hw_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  if(ret == 0){
    ctrl_reg1.pd = (uint8_t)val;
    ret = lps33hw_write_reg(ctx, LPS33HW_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  }
  return ret;
}

/**
  * @brief  Output data rate selection.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of odr in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_data_rate_get(stmdev_ctx_t *ctx, lps33hw_odr_t *val)
{
  lps33hw_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  switch (ctrl_reg1.odr){
    case LPS33HW_POWER_DOWN:
      *val = LPS33HW_POWER_DOWN;
      break;
    case LPS33HW_ODR_P1_Hz_T1_Hz:
      *val = LPS33HW_ODR_P1_Hz_T1_Hz;
      break;
    case LPS33HW_ODR_P7_Hz_T1_Hz:
      *val = LPS33HW_ODR_P7_Hz_T1_Hz;
      break;
    case LPS33HW_ODR_P12_Hz_T1_Hz:
      *val = LPS33HW_ODR_P12_Hz_T1_Hz;
      break;
    case LPS33HW_ODR_P25_Hz_T1_Hz:
      *val = LPS33HW_ODR_P25_Hz_T1_Hz;
      break;
    case LPS33HW_ODR_P7_Hz_T7_Hz:
      *val = LPS33HW_ODR_P7_Hz_T7_Hz;
      break;
      case LPS33HW_ODR_P12_Hz_T12_Hz:
      *val = LPS33HW_ODR_P12_Hz_T12_Hz;
      break;
    case LPS33HW_ODR_P25_Hz_T25_Hz:
      *val = LPS33HW_ODR_P25_Hz_T25_Hz;
      break;
    default:
      *val = LPS33HW_ODR_P1_Hz_T1_Hz;
      break;
  }

  return ret;
}

/**
  * @brief  One-shot mode. Device perform a single measure.[set] 
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of one_shot in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_one_shoot_trigger_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps33hw_ctrl_reg2_t ctrl_reg2;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  if(ret == 0){
    ctrl_reg2.one_shot = val;
    ret = lps33hw_write_reg(ctx, LPS33HW_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  }
  return ret;
}

/**
  * @brief  One-shot mode. Device perform a single measure.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of one_shot in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_one_shoot_trigger_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33hw_ctrl_reg2_t ctrl_reg2;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  *val = ctrl_reg2.one_shot;

  return ret;
}

/**
  * @brief  pressure_ref:   The Reference pressure value is a 24-bit data
  *         expressed as 2’s complement. The value is used when AUTOZERO
  *         or AUTORIFP function is enabled.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_pressure_ref_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret =  lps33hw_write_reg(ctx, LPS33HW_REF_P_XL, buff, 3);
  return ret;
}

/**
  * @brief  pressure_ref:   The Reference pressure value is a 24-bit data
  *         expressed as 2’s complement. The value is used when AUTOZERO
  *         or AUTORIFP function is enabled.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_pressure_ref_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret =  lps33hw_read_reg(ctx, LPS33HW_REF_P_XL, buff, 3);
  return ret;
}



/**
  * @brief  Pressure data available.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of p_da in reg STATUS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_press_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33hw_status_t status;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_STATUS, (uint8_t*)&status, 1);
  *val = status.p_da;

  return ret;
}

/**
  * @brief  Temperature data available.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of t_da in reg STATUS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_temp_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33hw_status_t status;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_STATUS, (uint8_t*)&status, 1);
  *val = status.t_da;

  return ret;
}

/**
  * @brief  Pressure data overrun.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of p_or in reg STATUS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_press_data_ovr_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33hw_status_t status;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_STATUS, (uint8_t*)&status, 1);
  *val = status.p_or;

  return ret;
}

/**
  * @brief  Temperature data overrun.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of t_or in reg STATUS
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_temp_data_ovr_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33hw_status_t status;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_STATUS, (uint8_t*)&status, 1);
  *val = status.t_or;

  return ret;
}

/**
  * @brief  Pressure output value[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_pressure_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret =  lps33hw_read_reg(ctx, LPS33HW_PRESS_OUT_XL, buff, 3);
  return ret;
}

/**
  * @brief  temperature_raw:   Temperature output value[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that stores data read.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_temperature_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret =  lps33hw_read_reg(ctx, LPS33HW_TEMP_OUT_L, (uint8_t*) buff, 2);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    LPS33HW_common
  * @brief       This section group common usefull functions
  * @{
  *
  */

/**
  * @brief  Device Who am I[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret =  lps33hw_read_reg(ctx, LPS33HW_WHO_AM_I, (uint8_t*) buff, 1);
  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of swreset in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_reset_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps33hw_ctrl_reg2_t ctrl_reg2;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  if(ret == 0){
    ctrl_reg2.swreset = val;
    ret = lps33hw_write_reg(ctx, LPS33HW_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  }
  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of swreset in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_reset_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33hw_ctrl_reg2_t ctrl_reg2;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  *val = ctrl_reg2.swreset;

  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of boot in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_boot_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps33hw_ctrl_reg2_t ctrl_reg2;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  if(ret == 0){
    ctrl_reg2.boot = val;
    ret = lps33hw_write_reg(ctx, LPS33HW_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  }
  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of boot in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_boot_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33hw_ctrl_reg2_t ctrl_reg2;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  *val = ctrl_reg2.boot;

  return ret;
}





/**
  * @}
  *
  */

/**
  * @defgroup    LPS33HW_interrupts
  * @brief       This section group all the functions that manage interrupts
  * @{
  *
  */

/**
  * @brief  Enable interrupt generation on pressure low/high event.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of pe in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_sign_of_int_threshold_set(stmdev_ctx_t *ctx,
                                           lps33hw_pe_t val)
{
  lps33hw_interrupt_cfg_t interrupt_cfg;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_INTERRUPT_CFG,
                         (uint8_t*)&interrupt_cfg, 1);
  if(ret == 0){
    interrupt_cfg.pe = (uint8_t)val;
    ret = lps33hw_write_reg(ctx, LPS33HW_INTERRUPT_CFG,
                            (uint8_t*)&interrupt_cfg, 1);
  }
  return ret;
}

/**
  * @brief  Enable interrupt generation on pressure low/high event.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of pe in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_sign_of_int_threshold_get(stmdev_ctx_t *ctx,
                                           lps33hw_pe_t *val)
{
  lps33hw_interrupt_cfg_t interrupt_cfg;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_INTERRUPT_CFG,
                         (uint8_t*)&interrupt_cfg, 1);
  switch (interrupt_cfg.pe){
    case LPS33HW_NO_THRESHOLD:
      *val = LPS33HW_NO_THRESHOLD;
      break;
    case LPS33HW_POSITIVE:
      *val = LPS33HW_POSITIVE;
      break;
    case LPS33HW_NEGATIVE:
      *val = LPS33HW_NEGATIVE;
      break;
    case LPS33HW_BOTH:
      *val = LPS33HW_BOTH;
      break;
    default:
      *val = LPS33HW_NO_THRESHOLD;
      break;
  }
  return ret;
}

/**
  * @brief  Interrupt request to the INT_SOURCE (25h) register
  *         mode (pulsed / latched) [set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of lir in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_int_notification_mode_set(stmdev_ctx_t *ctx,
                                           lps33hw_lir_t val)
{
  lps33hw_interrupt_cfg_t interrupt_cfg;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_INTERRUPT_CFG,
                         (uint8_t*)&interrupt_cfg, 1);
  if(ret == 0){
    interrupt_cfg.lir = (uint8_t)val;
    ret = lps33hw_write_reg(ctx, LPS33HW_INTERRUPT_CFG,
                            (uint8_t*)&interrupt_cfg, 1);
  }
  return ret;
}

/**
  * @brief   Interrupt request to the INT_SOURCE (25h) register
  *          mode (pulsed / latched) [get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of lir in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_int_notification_mode_get(stmdev_ctx_t *ctx,
                                          lps33hw_lir_t *val)
{
  lps33hw_interrupt_cfg_t interrupt_cfg;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_INTERRUPT_CFG,
                         (uint8_t*)&interrupt_cfg, 1);
  switch (interrupt_cfg.lir){
    case LPS33HW_INT_PULSED:
      *val = LPS33HW_INT_PULSED;
      break;
    case LPS33HW_INT_LATCHED:
      *val = LPS33HW_INT_LATCHED;
      break;
    default:
      *val = LPS33HW_INT_PULSED;
      break;
  }
  return ret;
}

/**
  * @brief  Enable interrupt generation.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of diff_en in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_int_generation_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lps33hw_interrupt_cfg_t interrupt_cfg;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_INTERRUPT_CFG,
                         (uint8_t*)&interrupt_cfg, 1);
  if(ret == 0){
    interrupt_cfg.diff_en = val;
    ret = lps33hw_write_reg(ctx, LPS33HW_INTERRUPT_CFG,
                            (uint8_t*)&interrupt_cfg, 1);
  }
  return ret;
}

/**
  * @brief  Enable interrupt generation.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of diff_en in reg INTERRUPT_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_int_generation_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33hw_interrupt_cfg_t interrupt_cfg;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_INTERRUPT_CFG,
                         (uint8_t*)&interrupt_cfg, 1);
  *val = interrupt_cfg.diff_en;

  return ret;
}

/**
  * @brief  User-defined threshold value for pressure interrupt event[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that contains data to write
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_int_threshold_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret =  lps33hw_write_reg(ctx, LPS33HW_THS_P_L, (uint8_t*) buff, 2);
  return ret;
}

/**
  * @brief  User-defined threshold value for pressure interrupt event[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  buff   Buffer that stores data read
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_int_threshold_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret =  lps33hw_read_reg(ctx, LPS33HW_THS_P_L, (uint8_t*) buff, 2);
  return ret;
}

/**
  * @brief  Data signal on INT_DRDY pin control bits.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of int_s in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_int_pin_mode_set(stmdev_ctx_t *ctx, lps33hw_int_s_t val)
{
  int32_t ret = 0;

  return ret;
}

/**
  * @brief  Data signal on INT_DRDY pin control bits.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of int_s in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_int_pin_mode_get(stmdev_ctx_t *ctx, lps33hw_int_s_t *val)
{ int32_t ret = 0;
  
  return ret;
}

/**
  * @brief  Push-pull/open drain selection on interrupt pads.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of pp_od in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_pin_mode_set(stmdev_ctx_t *ctx, lps33hw_pp_od_t val)
{
  lps33hw_ctrl_reg3_t ctrl_reg3;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_CTRL_REG3, (uint8_t*)&ctrl_reg3, 1);
  if(ret == 0){
    ctrl_reg3.pp_od = (uint8_t)val;
    ret = lps33hw_write_reg(ctx, LPS33HW_CTRL_REG3, (uint8_t*)&ctrl_reg3, 1);
  }
  return ret;
}

/**
  * @brief  Push-pull/open drain selection on interrupt pads.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of pp_od in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_pin_mode_get(stmdev_ctx_t *ctx, lps33hw_pp_od_t *val)
{
  lps33hw_ctrl_reg3_t ctrl_reg3;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_CTRL_REG3, (uint8_t*)&ctrl_reg3, 1);
  switch (ctrl_reg3.pp_od){
    case LPS33HW_PUSH_PULL:
      *val = LPS33HW_PUSH_PULL;
      break;
    case LPS33HW_OPEN_DRAIN:
      *val = LPS33HW_OPEN_DRAIN;
      break;
    default:
      *val = LPS33HW_PUSH_PULL;
      break;
  }
  return ret;
}

/**
  * @brief  Interrupt active-high/low.[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of int_h_l in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_int_polarity_set(stmdev_ctx_t *ctx, lps33hw_int_h_l_t val)
{
  lps33hw_ctrl_reg3_t ctrl_reg3;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_CTRL_REG3, (uint8_t*)&ctrl_reg3, 1);
  if(ret == 0){
    ctrl_reg3.int_h_l = (uint8_t)val;
    ret = lps33hw_write_reg(ctx, LPS33HW_CTRL_REG3, (uint8_t*)&ctrl_reg3, 1);
  }
  return ret;
}

/**
  * @brief  Interrupt active-high/low.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of int_h_l in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_int_polarity_get(stmdev_ctx_t *ctx, lps33hw_int_h_l_t *val)
{
  lps33hw_ctrl_reg3_t ctrl_reg3;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_CTRL_REG3, (uint8_t*)&ctrl_reg3, 1);
  switch (ctrl_reg3.int_h_l){
    case LPS33HW_ACTIVE_HIGH:
      *val = LPS33HW_ACTIVE_HIGH;
      break;
    case LPS33HW_ACTIVE_LOW:
      *val = LPS33HW_ACTIVE_LOW;
      break;
    default:
      *val = LPS33HW_ACTIVE_HIGH;
      break;
  }
  return ret;
}

/**
  * @brief  Interrupt source register[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Register INT_SOURCE
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_int_source_get(stmdev_ctx_t *ctx, lps33hw_int_source_t *val)
{
  int32_t ret;
  ret =  lps33hw_read_reg(ctx, LPS33HW_INT_SOURCE, (uint8_t*) val, 1);
  return ret;
}

/**
  * @brief  Differential pressure high interrupt flag.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of ph in reg INT_SOURCE
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_int_on_press_high_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33hw_int_source_t int_source;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_INT_SOURCE, (uint8_t*)&int_source, 1);
  *val = int_source.ph;

  return ret;
}

/**
  * @brief  Differential pressure low interrupt flag.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of pl in reg INT_SOURCE
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_int_on_press_low_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33hw_int_source_t int_source;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_INT_SOURCE, (uint8_t*)&int_source, 1);
  *val = int_source.pl;

  return ret;
}

/**
  * @brief  Interrupt active flag.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of ia in reg INT_SOURCE
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_interrupt_event_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  lps33hw_int_source_t int_source;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_INT_SOURCE, (uint8_t*)&int_source, 1);
  *val = int_source.ia;

  return ret;
}



/**
  * @brief  SPI Serial Interface Mode selection.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of sim in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_spi_mode_get(stmdev_ctx_t *ctx, lps33hw_sim_t *val)
{
  lps33hw_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = lps33hw_read_reg(ctx, LPS33HW_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  switch (ctrl_reg1.sim){
    case LPS33HW_SPI_4_WIRE:
      *val = LPS33HW_SPI_4_WIRE;
      break;
    case LPS33HW_SPI_3_WIRE:
      *val = LPS33HW_SPI_3_WIRE;
      break;
    default:
      *val = LPS33HW_SPI_4_WIRE;
      break;
  }
  return ret;
}


/**
  * @brief  Disable I2C interface.[get]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Get the values of i2c_dis in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_i2c_interface_get(stmdev_ctx_t *ctx, lps33hw_i2c_dis_t *val)
{
  int32_t ret = 0;

  return ret;
}

/**
  * @brief  Register address automatically incremented during a
  *         multiple byte access with a serial interface (I2C or SPI).[set]
  *
  * @param  ctx    Read / write interface definitions
  * @param  val    Change the values of if_add_inc in reg CTRL_REG2
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t lps33hw_auto_add_inc_set(stmdev_ctx_t *ctx, uint8_t val)
{
  int32_t ret = 0;

  return ret;
}

/**
  * @}
  *
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
