/*
 * cli_port.cpp
 *
 *  Created on: 12 ����. 2019 �.
 *      Author: d.semenyuk
 */

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "cli_config.h"
#include "cli_port.h"
#include "cli_base.h"
#include "cli_cmd.h"
#include "FreeRTOS.h"
#include "main.h"

using namespace cli;

extern UART_HandleTypeDef huart3;
SemaphoreHandle_t SerialMutex;
//#####################################################################################################

void cli::cli_sleep(uint32_t time_ms)
{
	if (time_ms == 0)
	{
		return;
	}
	if (time_ms == CLI_MAX_DELAY)
	{
		time_ms = 0;
	}
	vTaskDelay(time_ms);
}
//#####################################################################################################
//	INPUT/OUTPUT
//	����� ���������� ����������� CLI ������ ������� �����/������ ��� ��������� ���������� �����������
//-----------------------------------------------------------------------------------------------------
//	USB

extern "C" void DbgPutChar(char val)
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&val, 1, 1000);
}

extern "C" int DbgPrintf(const char *format, ...)
{
	static char printf_buff[CLI_PRINTF_WORK_BUFFER_SIZE] = {};
	__Va_list ap;
	xSemaphoreTake(SerialMutex, portMAX_DELAY);

	va_start(ap, format);
	int len = vsprintf(printf_buff, format, ap);
	va_end(ap);

	if (len < 0) //???? vsprintf() ??????????? ? ???????
	{
		const char *msg_printf_error = {"\n\rERROR: cli_printf(): vsprintf() return error;\n\r"};
		HAL_UART_Transmit(&huart3, (uint8_t *)msg_printf_error, sizeof(msg_printf_error), 1000);
	}
	else if (len > sizeof(printf_buff))
	{
		const char *msg_printf_error = {"\n\rERROR: cli_printf(): vsprintf() need larger working buffer size!\n\r"};
		HAL_UART_Transmit(&huart3, (uint8_t *)msg_printf_error, sizeof(msg_printf_error), 1000);
	}
	else
	{
		for (int i = 0; i < len; i++)
		{
			if (printf_buff[i] == '\n')
				DbgPutChar('\r');
			DbgPutChar(printf_buff[i]);
		}
	}
	xSemaphoreGive(SerialMutex);
	return len;
}

extern "C" bool DbgReadChar(char *const val, uint32_t timeout)
{

	if (HAL_UART_Receive(&huart3, (uint8_t *)val, 1, timeout) == HAL_OK)
	{
		return true;
	}
	return false;
}
extern "C" int DbgScanf(const char *format, ...)
{
	static char scanf_buff[CLI_SCANF_WORK_BUFFER_SIZE];
	int result;
	uint32_t buf_index = 0;

	xSemaphoreTake(SerialMutex, portMAX_DELAY);

	while (1)
	{
		char tmp;
		DbgReadChar(&tmp, CLI_MAX_DELAY);

		if (tmp == '\r' || tmp == '\n')
		{
			break;
		}
		DbgPutChar(tmp);
		if (tmp == '\b')
		{
			if (buf_index != 0)
			{
				buf_index--;
				DbgPutChar(' ');
				DbgPutChar('\b');
			}
			continue;
		}
		scanf_buff[buf_index++] = tmp;
	}
	scanf_buff[buf_index] = '\0';
	__Va_list ap;
	va_start(ap, format);
	result = vsscanf(scanf_buff, format, ap);
	va_end(ap);
	xSemaphoreGive(SerialMutex);
	return result;
}

const TCLI_IO DBG_CLI_IOStruct = {
	DbgPutChar,
	DbgReadChar,
	DbgPrintf,
	DbgScanf};

//#####################################################################################################
//	CLI
//	����� ��������� ���������� CLI

//-----------------------------------------------------------------------------------------------------
//	Debug CLI

TT_CLI<CLI_MAX_INPUT_STR_LEN, CLI_MAX_HISTORY_STR>
	DBG_CLI_UART(&DBG_CLI_IOStruct, INVITE_STRING, CLI_CommandsParser);

//#####################################################################################################
//	TASKS
//	����� ��������� �������� ��������� CLI. �������������� ������ CLI � ��������� ��������

//-----------------------------------------------------------------------------------------------------

// Serial
extern "C" void DBG_CLI_Serial_Task()
{
	SerialMutex = xSemaphoreCreateMutex();
	while (1)
	{
		DBG_CLI_UART.Process();
	}
}