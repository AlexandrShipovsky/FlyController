/*
 * cli_cmd.cpp
 *
 *  Created on: 03 ���. 2019 �.
 *      Author: d.semenyuk
 */

#include <stdint.h>
#include <string.h>
#include "cli_config.h"
#include "cli_port.h"
#include "cli_base.h"
#include "cli_cmd.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

#include "stm32f7xx_hal.h"

#include "IMU.h"

void CLI_CommandsParser(const TCLI_IO *const io, char *ps, CLI_InputStrLen_t len)
{

	CLI_IF_CMD("LED", "Commands for switching LD2") // ������� ���������� ����������� PC13
	{
		CLI_NEXT_WORD();
		CLI_IF_CMD("ON", "LED ON")
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
			ok;
			return;
		}
		CLI_IF_CMD("OFF", "LED OFF")
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
			ok;
			return;
		}
		CLI_IF_CMD("SWITCH", "LED SWITCH")
		{
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
			ok;
			return;
		}
		CLI_INVALID_KEYWORD();
		return;
	}

	CLI_IF_CMD("I2CSCAN", "Scanning i2c avalible adress")
	{
		extern I2C_HandleTypeDef hi2c4;
		if (HAL_I2C_IsDeviceReady(&hi2c4, H3LIS331DL_I2C_ADD_L, 1, 100) == HAL_OK)
		{
			DbgPrintf("Accel I2C adr = %i (HEX: 0x%X)\n\r", H3LIS331DL_I2C_ADD_L, H3LIS331DL_I2C_ADD_L);
		}

		if (HAL_I2C_IsDeviceReady(&hi2c4, A3G4250D_I2C_ADD_L, 1, 100) == HAL_OK)
		{
			DbgPrintf("Gyro I2C adr = %i (HEX: 0x%X)\n\r", A3G4250D_I2C_ADD_L, A3G4250D_I2C_ADD_L);
		}

		if (HAL_I2C_IsDeviceReady(&hi2c4, LIS3MDL_I2C_ADD_L, 1, 100) == HAL_OK)
		{
			DbgPrintf("Compass I2C adr = %i (HEX: 0x%X)\n\r", LIS3MDL_I2C_ADD_L, LIS3MDL_I2C_ADD_L);
		}

		if (HAL_I2C_IsDeviceReady(&hi2c4, LPS331_I2C_ADD_L, 1, 100) == HAL_OK)
		{
			DbgPrintf("Barometer I2C adr = %i (HEX: 0x%X)\n\r", LPS331_I2C_ADD_L, LPS331_I2C_ADD_L);
		}
		return;
	}

	CLI_IF_CMD("LIS3MDLINIT", "Init lis3mdl")
	{

		return;
	}

	//----------------------------------------------------------------------------------
	CLI_UNKNOWN_COMMAND();
}}