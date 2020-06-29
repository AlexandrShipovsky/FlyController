/*
 ******************************************************************************
 * @file    lps331_reg.h
 * @author  Sensors Software Solution Team (modificated Clear Sky LLC)
 * @brief   This file contains all the functions prototypes for the
 *          lps331_reg.c driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LPS331_REGS_H
#define LPS331_REGS_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/** @addtogroup LPS331
  * @{
  *
  */

/** @defgroup STMicroelectronics sensors common types
  * @{
  *
  */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

typedef struct{
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
} bitwise_t;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*stmdev_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_t;

/**
  * @}
  *
  */

#endif /* MEMS_SHARED_TYPES */

#ifndef MEMS_UCF_SHARED_TYPES
#define MEMS_UCF_SHARED_TYPES

/** @defgroup    Generic address-data structure definition
  * @brief       This structure is useful to load a predefined configuration
  *              of a sensor.
	*              You can create a sensor configuration by your own or using 
	*              Unico / Unicleo tools available on STMicroelectronics
	*              web site.
  *
  * @{
  *
  */

typedef struct {
  uint8_t address;
  uint8_t data;
} ucf_line_t;

/**
  * @}
  *
  */

#endif /* MEMS_UCF_SHARED_TYPES */

/**
  * @}
  *
  */


/** @defgroup LSM9DS1_Infos
  * @{
  *
  */

  /** I2C Device Address 8 bit format: if SA0=0 -> 0xB9 if SA0=1 -> 0xBB **/
#define LPS331_I2C_ADD_H   0xBBU
#define LPS331_I2C_ADD_L   0xB9U

/** Device Identification (Who am I) **/
#define LPS331_ID            0xBBU

/**
  * @}
  *
  */

#define LPS33HW_INTERRUPT_CFG  0x23U
typedef struct {
  uint8_t pe               : 2; /* ple + phe -> pe */
  uint8_t lir              : 1;
  uint8_t diff_en          : 1;
  uint8_t reset_az         : 1;
  uint8_t autozero         : 1;
  uint8_t reset_arp        : 1;
  uint8_t autorifp         : 1;
} lps33hw_interrupt_cfg_t;

#define LPS33HW_THS_P_L        0x25U
#define LPS33HW_THS_P_H        0x26U
#define LPS33HW_WHO_AM_I       0x0FU
#define LPS33HW_CTRL_REG1      0x20U
typedef struct {
  uint8_t sim              : 1;
  uint8_t delta_en         : 1;
  uint8_t dbdu             : 1; 
  uint8_t diff_en          : 1;
  uint8_t odr              : 3;
  uint8_t pd               : 1;
} lps33hw_ctrl_reg1_t;

#define LPS33HW_CTRL_REG2      0x21U
typedef struct {
  uint8_t one_shot         : 1;
  uint8_t auto_zero        : 1;
  uint8_t swreset          : 1;
  uint8_t reserved         : 4;
  uint8_t boot             : 1;
} lps33hw_ctrl_reg2_t;

#define LPS33HW_CTRL_REG3      0x22U
typedef struct {
  uint8_t int1_s           : 3;
  uint8_t int2_s           : 3;
  uint8_t pp_od            : 1;
  uint8_t int_h_l          : 1;
} lps33hw_ctrl_reg3_t;

#define LPS33HW_REF_P_XL       0x08U
#define LPS33HW_REF_P_L        0x09U
#define LPS33HW_REF_P_H        0x0AU

#define LPS33HW_RES_CONF       0x10U
typedef struct {
  uint8_t avgp             : 4;
  uint8_t avgt             : 3;
  uint8_t rfu             : 1;
} lps33hw_res_conf_t;

#define LPS33HW_INT_SOURCE     0x24U
typedef struct {
  uint8_t ph               : 1;
  uint8_t pl               : 1;
  uint8_t ia               : 1;
  uint8_t not_used_01      : 5;
} lps33hw_int_source_t;

#define LPS33HW_STATUS         0x27U
typedef struct {
  uint8_t t_da             : 1;
  uint8_t p_da             : 1;
  uint8_t not_used_02      : 2;
  uint8_t t_or             : 1;
  uint8_t p_or             : 1;
  uint8_t not_used_01      : 2;
} lps33hw_status_t;

#define LPS33HW_PRESS_OUT_XL   0x28U
#define LPS33HW_PRESS_OUT_L    0x29U
#define LPS33HW_PRESS_OUT_H    0x2AU
#define LPS33HW_TEMP_OUT_L     0x2BU
#define LPS33HW_TEMP_OUT_H     0x2CU

/**
  * @defgroup LPS33HW_Register_Union
  * @brief    This union group all the registers that has a bit-field
  *           description.
  *           This union is useful but not need by the driver.
  *
  *           REMOVING this union you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  *
  * @{
  *
  */

typedef union{
  lps33hw_interrupt_cfg_t      interrupt_cfg;
  lps33hw_ctrl_reg1_t          ctrl_reg1;
  lps33hw_ctrl_reg2_t          ctrl_reg2;
  lps33hw_ctrl_reg3_t          ctrl_reg3;
  lps33hw_res_conf_t           res_conf;
  lps33hw_int_source_t         int_source;
  lps33hw_status_t             status;
  bitwise_t                    bitwise;
  uint8_t                      byte;
} lps33hw_reg_t;

/**
  * @}
  *
  */

int32_t lps33hw_read_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                         uint16_t len);
int32_t lps33hw_write_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data,
                          uint16_t len);

extern float_t lps33hw_from_lsb_to_hpa(int32_t lsb);
extern float_t lps33hw_from_lsb_to_degc(int16_t lsb);

int32_t lps33hw_autozero_rst_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps33hw_autozero_rst_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33hw_autozero_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps33hw_autozero_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33hw_pressure_snap_rst_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps33hw_pressure_snap_rst_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33hw_pressure_snap_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps33hw_pressure_snap_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33hw_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps33hw_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LPS33HW_LPF_ODR_DIV_2  = 0,
  LPS33HW_LPF_ODR_DIV_9  = 2,
  LPS33HW_LPF_ODR_DIV_20 = 3,
} lps33hw_lpfp_t;
int32_t lps33hw_low_pass_filter_mode_set(stmdev_ctx_t *ctx,
                                         lps33hw_lpfp_t val);
int32_t lps33hw_low_pass_filter_mode_get(stmdev_ctx_t *ctx,
                                         lps33hw_lpfp_t *val);

typedef enum {
  LPS33HW_POWER_DOWN  = 0,
  LPS33HW_ODR_P1_Hz_T1_Hz    = 1,
  LPS33HW_ODR_P7_Hz_T1_Hz    = 2,
  LPS33HW_ODR_P12_Hz_T1_Hz   = 3,
  LPS33HW_ODR_P25_Hz_T1_Hz   = 4,
  LPS33HW_ODR_P7_Hz_T7_Hz    = 5,
  LPS33HW_ODR_P12_Hz_T12_Hz  = 6,
  LPS33HW_ODR_P25_Hz_T25_Hz  = 7,
} lps33hw_odr_t;
int32_t lps33hw_data_rate_set(stmdev_ctx_t *ctx, lps33hw_odr_t val);
int32_t lps33hw_data_rate_get(stmdev_ctx_t *ctx, lps33hw_odr_t *val);

int32_t lps33hw_one_shoot_trigger_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps33hw_one_shoot_trigger_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33hw_pressure_ref_set(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lps33hw_pressure_ref_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lps33hw_pressure_offset_set(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lps33hw_pressure_offset_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lps33hw_press_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33hw_temp_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33hw_press_data_ovr_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33hw_temp_data_ovr_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33hw_pressure_raw_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lps33hw_temperature_raw_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lps33hw_low_pass_rst_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lps33hw_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lps33hw_reset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps33hw_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33hw_boot_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps33hw_boot_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33hw_boot_status_get(stmdev_ctx_t *ctx, uint8_t *val);


typedef enum {
  LPS33HW_NO_THRESHOLD = 0,
  LPS33HW_POSITIVE     = 1,
  LPS33HW_NEGATIVE     = 2,
  LPS33HW_BOTH         = 3,
} lps33hw_pe_t;
int32_t lps33hw_sign_of_int_threshold_set(stmdev_ctx_t *ctx,
                                          lps33hw_pe_t val);
int32_t lps33hw_sign_of_int_threshold_get(stmdev_ctx_t *ctx,
                                          lps33hw_pe_t *val);

typedef enum {
  LPS33HW_INT_PULSED  = 0,
  LPS33HW_INT_LATCHED = 1,
} lps33hw_lir_t;
int32_t lps33hw_int_notification_mode_set(stmdev_ctx_t *ctx,
                                          lps33hw_lir_t val);
int32_t lps33hw_int_notification_mode_get(stmdev_ctx_t *ctx,
                                          lps33hw_lir_t *val);

int32_t lps33hw_int_generation_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps33hw_int_generation_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33hw_int_threshold_set(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lps33hw_int_threshold_get(stmdev_ctx_t *ctx, uint8_t *buff);

typedef enum {
  LPS33HW_DRDY_OR_FIFO_FLAGS = 0,
  LPS33HW_HIGH_PRES_INT      = 1,
  LPS33HW_LOW_PRES_INT       = 2,
  LPS33HW_EVERY_PRES_INT     = 3,
} lps33hw_int_s_t;
int32_t lps33hw_int_pin_mode_set(stmdev_ctx_t *ctx, lps33hw_int_s_t val);
int32_t lps33hw_int_pin_mode_get(stmdev_ctx_t *ctx, lps33hw_int_s_t *val);

int32_t lps33hw_drdy_on_int_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps33hw_drdy_on_int_get(stmdev_ctx_t *ctx, uint8_t *val);


typedef enum {
  LPS33HW_PUSH_PULL  = 0,
  LPS33HW_OPEN_DRAIN = 1,
} lps33hw_pp_od_t;
int32_t lps33hw_pin_mode_set(stmdev_ctx_t *ctx, lps33hw_pp_od_t val);
int32_t lps33hw_pin_mode_get(stmdev_ctx_t *ctx, lps33hw_pp_od_t *val);

typedef enum {
  LPS33HW_ACTIVE_HIGH = 0,
  LPS33HW_ACTIVE_LOW = 1,
} lps33hw_int_h_l_t;
int32_t lps33hw_int_polarity_set(stmdev_ctx_t *ctx, lps33hw_int_h_l_t val);
int32_t lps33hw_int_polarity_get(stmdev_ctx_t *ctx, lps33hw_int_h_l_t *val);

int32_t lps33hw_int_source_get(stmdev_ctx_t *ctx, lps33hw_int_source_t *val);

int32_t lps33hw_int_on_press_high_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33hw_int_on_press_low_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33hw_interrupt_event_get(stmdev_ctx_t *ctx, uint8_t *val);


typedef enum {
  LPS33HW_BYPASS_MODE           = 0,
  LPS33HW_FIFO_MODE             = 1,
  LPS33HW_STREAM_MODE           = 2,
  LPS33HW_STREAM_TO_FIFO_MODE   = 3,
  LPS33HW_BYPASS_TO_STREAM_MODE = 4,
  LPS33HW_DYNAMIC_STREAM_MODE   = 6,
  LPS33HW_BYPASS_TO_FIFO_MODE   = 7,
} lps33hw_f_mode_t;

typedef enum {
  LPS33HW_SPI_4_WIRE = 0,
  LPS33HW_SPI_3_WIRE = 1,
} lps33hw_sim_t;
int32_t lps33hw_spi_mode_set(stmdev_ctx_t *ctx, lps33hw_sim_t val);
int32_t lps33hw_spi_mode_get(stmdev_ctx_t *ctx, lps33hw_sim_t *val);

typedef enum {
  LPS33HW_I2C_ENABLE = 0,
  LPS33HW_I2C_DISABLE = 1,
} lps33hw_i2c_dis_t;
int32_t lps33hw_i2c_interface_set(stmdev_ctx_t *ctx, lps33hw_i2c_dis_t val);
int32_t lps33hw_i2c_interface_get(stmdev_ctx_t *ctx, lps33hw_i2c_dis_t *val);

int32_t lps33hw_auto_add_inc_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps33hw_auto_add_inc_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps33hw_power_set(stmdev_ctx_t *ctx, uint8_t val);
/**
  *@}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /* LPS33HW_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
