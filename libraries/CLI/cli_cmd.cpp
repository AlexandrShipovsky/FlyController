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

#include "stm32f3xx_hal.h"
#include "MotorDC.h"
#include "encoder.h"
#include "vbat.h"
#include "pid.h"

void CLI_CommandsParser(const TCLI_IO *const io, char *ps, CLI_InputStrLen_t len)
{

	CLI_IF_CMD("LED", "Commands for switching LED PC13") // ������� ���������� ����������� PC13
	{
		CLI_NEXT_WORD();
		CLI_IF_CMD("ON", "LED ON")
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			ok;
			return;
		}
		CLI_IF_CMD("OFF", "LED OFF")
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			ok;
			return;
		}
		CLI_IF_CMD("SWITCH", "LED SWITCH")
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			ok;
			return;
		}
		CLI_INVALID_KEYWORD();
		return;
	}

	CLI_IF_CMD("CANTX", "Send CAN")
	{
		CAN_TxHeaderTypeDef TxHeader;
		extern CAN_HandleTypeDef hcan;

		TxHeader.DLC = 2;
		TxHeader.StdId = 0x00FF;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.TransmitGlobalTime = DISABLE;

		uint8_t buf[2];
		uint32_t TxMailBox;

		buf[0] = 0x11;
		buf[1] = 0x22;

		if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, buf, &TxMailBox) != HAL_OK)
		{
			Error_Handler();
		}
		return;
	}

	CLI_IF_CMD("ROT", "Rotation DC")
	{
		float pulse;
		int16_t pos;
		extern MotorDCTypeDef MotorRoll;
		extern MotorDCTypeDef MotorPitch;
		extern EncTypeDef EncRoll;
		extern EncTypeDef EncPitch;

		CLI_NEXT_WORD();

		// ROLL
		CLI_IF_CMD("ROLL", "Rotation roll motor")
		{
			CLI_SCAN_PARAM("%f", pulse, "Pulse = ");
			CLI_SCAN_PARAM("%i", pos, "Position = ");

			DbgPrintf("Value = %d\n\r", pos);
			MotorRoll.pulse = pulse;

			if (pos > GetEnc(&EncRoll))
			{
				MotorRoll.DirOfRot = DIRECT_ROTATION;
				rotation(&MotorRoll);
				while (pos > GetEnc(&EncRoll))
				{
					DbgPrintf("Enc value = %d\n\r", GetEnc(&EncRoll));
				}
				StopRotation(&MotorRoll);
				return;
			}
			if (pos < GetEnc(&EncRoll))
			{
				MotorRoll.DirOfRot = REVERSE_ROTATION;
				rotation(&MotorRoll);
				while (pos < GetEnc(&EncRoll))
				{
					DbgPrintf("Enc value = %d\n\r", GetEnc(&EncRoll));
				}
				StopRotation(&MotorRoll);
				return;
			}

			return;
		}

		// PITCH
		CLI_IF_CMD("PITCH", "Rotation pitch motor")
		{
			CLI_SCAN_PARAM("%f", pulse, "Pulse = ");
			CLI_SCAN_PARAM("%d", pos, "Position = ");

			DbgPrintf("Value = %i\n\r", pos);

			MotorPitch.pulse = pulse;

			if (pos > GetEnc(&EncPitch))
			{
				MotorPitch.DirOfRot = DIRECT_ROTATION;
				rotation(&MotorPitch);
				while (pos > GetEnc(&EncPitch))
				{
					DbgPrintf("\n\rEnc value = %d\n\r", GetEnc(&EncPitch));
				}
				StopRotation(&MotorPitch);
				return;
			}
			if (pos < GetEnc(&EncPitch))
			{
				MotorPitch.DirOfRot = REVERSE_ROTATION;
				rotation(&MotorPitch);
				while (pos < GetEnc(&EncPitch))
				{
					DbgPrintf("\n\rEnc value = %d\n\r", GetEnc(&EncPitch));
				}
				StopRotation(&MotorPitch);
				return;
			}

			return;
		}
		CLI_INVALID_KEYWORD();
		return;
	}

	CLI_IF_CMD("STOPROT", "Rotation DC")
	{
		extern MotorDCTypeDef MotorRoll;
		extern MotorDCTypeDef MotorPitch;
		StopRotation(&MotorRoll);
		StopRotation(&MotorPitch);
		return;
	}

	CLI_IF_CMD("PITCH", "Get value of encoder")
	{
		extern EncTypeDef EncPitch;
		int16_t val = 13;
		val = GetEnc(&EncPitch);

		DbgPrintf("\n\rEnc value = %d\n\r", val);
		return;
	}

	CLI_IF_CMD("ROLL", "Get value of encoder")
	{
		extern EncTypeDef EncRoll;
		int16_t val = 13;
		val = GetEnc(&EncRoll);

		DbgPrintf("\n\rEnc value = %d\n\r", val);
		return;
	}

	CLI_IF_CMD("VBAT", "Get votage battery")
	{
		extern vbatTypeDef vbat;
		;
		float voltage = 13.0;
		voltage = GetVoltageBat(&vbat);

		DbgPrintf("\n\rVotage battery = %f\n\r", voltage);
		return;
	}

	CLI_IF_CMD("STACK", "Get size stack of tasks")
	{
		char InfoStack[1024];

		vTaskList(InfoStack);
		DbgPrintf("\n\r");
		DbgPrintf(InfoStack);
		DbgPrintf("\n\r");
		return;
	}

	CLI_IF_CMD("TIMESTATS", "Get time in percent processor")
	{
		char InfoStack[1024];

		vTaskGetRunTimeStats(InfoStack);
		DbgPrintf("\n\r");
		DbgPrintf(InfoStack);
		DbgPrintf("\n\r");
		return;
	}

	CLI_IF_CMD("PIDPITCH", "Get inform pid of pitch")
	{
		extern pidTypeDef pidPitch;
		DbgPrintf("\n\r");
		DbgPrintf("ManipulVal = %f\n\r", pidPitch.ManipulVal);
		DbgPrintf("DirOfRot = %i\n\r", pidPitch.DirOfRot);
		DbgPrintf("Kp = %f\n\r", pidPitch.Kp);
		DbgPrintf("Ki = %f\n\r", pidPitch.Ki);
		DbgPrintf("Kd = %f\n\r", pidPitch.Kd);
		DbgPrintf("\n\r");
		return;
	}

	CLI_IF_CMD("PIDROLL", "Get inform pid of roll")
	{
		extern pidTypeDef pidRoll;
		DbgPrintf("\n\r");
		DbgPrintf("ManipulVal = %f\n\r", pidRoll.ManipulVal);
		DbgPrintf("DirOfRot = %i\n\r", pidRoll.DirOfRot);
		DbgPrintf("Set point = %i\n\r", pidRoll.SetPoint);
		DbgPrintf("Kp = %f\n\r", pidRoll.Kp);
		DbgPrintf("Ki = %f\n\r", pidRoll.Ki);
		DbgPrintf("Kd = %f\n\r", pidRoll.Kd);
		DbgPrintf("\n\r");
		return;
	}

	CLI_IF_CMD("SETZERO", "Get inform pid of roll")
	{
		extern EncTypeDef EncRoll;
		extern EncTypeDef EncPitch;
		SetZero(&EncPitch);
		SetZero(&EncRoll);
		return;
	}

	CLI_IF_CMD("MROLL", "Rotation roll with PID")
	{
		int16_t pos;
		extern pidTypeDef pidRoll;
		CLI_SCAN_PARAM("%d", pos, "");
		pidRoll.SetPoint = pos;
		return;
	}

	CLI_IF_CMD("MPITCH", "Rotation pitch with PID")
	{
		int16_t pos;
		extern pidTypeDef pidPitch;
		CLI_SCAN_PARAM("%d", pos, "");
		pidPitch.SetPoint = pos;
		return;
	}

	CLI_IF_CMD("KP", "Proportional koeficient PID")
	{
		uint32_t varible;
		extern pidTypeDef pidRoll;
		CLI_SCAN_PARAM("%i", varible, "");
		pidRoll.Kp = varible/100000.0;
		return;
	}

	CLI_IF_CMD("KI", "Integral koeficient PID")
	{
		uint32_t varible;
		extern pidTypeDef pidRoll;
		CLI_SCAN_PARAM("%i", varible, "");
		pidRoll.Ki = varible/100000.0;
		return;
	}

	CLI_IF_CMD("KD", "Differencial koeficient PID")
	{
		uint32_t varible;
		extern pidTypeDef pidRoll;
		CLI_SCAN_PARAM("%i", varible, "");
		pidRoll.Kd = varible/100000.0;
		return;
	}

	//----------------------------------------------------------------------------------
	CLI_UNKNOWN_COMMAND();
}
