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


void CLI_CommandsParser(const TCLI_IO *const io, char *ps, CLI_InputStrLen_t len)
{

	CLI_IF_CMD("LED", "Commands for switching LED PC13") // ������� ���������� ����������� PC13
	{
		CLI_NEXT_WORD();
		CLI_IF_CMD("ON", "LED ON")
		{
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
			ok;
			return;
		}
		CLI_IF_CMD("OFF", "LED OFF")
		{
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
			ok;
			return;
		}
		CLI_IF_CMD("SWITCH", "LED SWITCH")
		{
			HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
			ok;
			return;
		}
		CLI_INVALID_KEYWORD();
		return;
	}

	CLI_IF_CMD("CANTX", "Send CAN")
	{
		CAN_TxHeaderTypeDef TxHeader;
		extern CAN_HandleTypeDef hcan1;

		TxHeader.DLC = 2;
		TxHeader.StdId = 0x00FF;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.TransmitGlobalTime = DISABLE;

		uint8_t buf[2];
		uint32_t TxMailBox;

		buf[0] = 0x11;
		buf[1] = 0x22;

		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, buf, &TxMailBox) != HAL_OK)
		{
			Error_Handler();
		}
		return;
	}


	//----------------------------------------------------------------------------------
	CLI_UNKNOWN_COMMAND();
}
