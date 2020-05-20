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
#include "usbd_cdc_if.h"
#include "FreeRTOS.h"

using namespace cli;

extern uint8_t UserRxBufferFS[];
//SemaphoreHandle_t CliMutex;
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
	//HAL_Delay(time_ms);
	vTaskDelay(time_ms);
}
//#####################################################################################################
//	INPUT/OUTPUT
//	����� ���������� ����������� CLI ������ ������� �����/������ ��� ��������� ���������� �����������
//-----------------------------------------------------------------------------------------------------
//	USB

extern "C" void DbgUSBPutChar(char val)
{
	uint8_t result = USBD_OK;
	char str[2];
	str[0] = val;
	str[1] = '\0';
	
	while (1)
	{
		result = CDC_Transmit_FS((uint8_t *)str, strlen(str));
		if (result == USBD_OK)
		{
			
			break;
		}
		else if (result == USBD_FAIL)
		{
			
			//ERROR
		}
	}
}

extern "C" int DbgPrintf(const char *format, ...)
{
	static char USB_printf_buff[CLI_PRINTF_WORK_BUFFER_SIZE] = {};
	//xSemaphoreTake(CliMutex, 100);
	__Va_list ap;

	va_start(ap, format);
	int len = vsprintf(USB_printf_buff, format, ap);
	va_end(ap);

	if (len < 0) //???? vsprintf() ??????????? ? ???????
	{
		const char *msg_printf_error = {"\n\rERROR: cli_printf(): vsprintf() return error;\n\r"};
		CDC_Transmit_FS((uint8_t *)msg_printf_error, sizeof(msg_printf_error));
	}
	else if (len > sizeof(USB_printf_buff))
	{
		const char *msg_printf_error = {"\n\rERROR: cli_printf(): vsprintf() need larger working buffer size!\n\r"};
		CDC_Transmit_FS((uint8_t *)msg_printf_error, sizeof(msg_printf_error));
	}
	else
	{
		for (int i = 0; i < len; i++)
		{
			if (USB_printf_buff[i] == '\n')
				DbgUSBPutChar('\r');
			DbgUSBPutChar(USB_printf_buff[i]);
		}
	}
	//xSemaphoreGive(CliMutex);
	return len;
}

extern "C" bool DbgUSBReadChar(char *const val, uint32_t timeout)
{
	uint32_t j = 0;
	uint16_t i = 0;
	uint16_t APP_RX_DATA_SIZE = sizeof(&UserRxBufferFS);
	while (j < timeout)
	{
		if (UserRxBufferFS[i] != '\0')
		{
			*val = (char)UserRxBufferFS[i];
			UserRxBufferFS[i] = '\0';

			return true;
		}
		i++;
		j++;
		if (i == APP_RX_DATA_SIZE)
			i = 0;
	}
	return false;
}
extern "C" int DbgScanf(const char *format, ...)
{
	static char USB_scanf_buff[CLI_SCANF_WORK_BUFFER_SIZE];
	//xSemaphoreTake(CliMutex, 100);

	int result;
	uint32_t buf_index = 0;

	while (1)
	{
		char tmp;
		DbgUSBReadChar(&tmp, CLI_MAX_DELAY);

		if (tmp == '\r' || tmp == '\n')
		{
			//xSemaphoreGive(CliMutex);
			break;
		}
		DbgUSBPutChar(tmp);
		if (tmp == '\b')
		{
			if (buf_index != 0)
			{
				buf_index--;
				DbgUSBPutChar(' ');
				DbgUSBPutChar('\b');
			}
			continue;
		}
		USB_scanf_buff[buf_index++] = tmp;
	}
	USB_scanf_buff[buf_index] = '\0';
	__Va_list ap;
	va_start(ap, format);
	result = vsscanf(USB_scanf_buff, format, ap);
	va_end(ap);
	//xSemaphoreGive(CliMutex);
	return result;
}

const TCLI_IO DBG_CLI_IOStruct = {
	DbgUSBPutChar,
	DbgUSBReadChar,
	DbgPrintf,
	DbgScanf};

//#####################################################################################################
//	CLI
//	����� ��������� ���������� CLI

//-----------------------------------------------------------------------------------------------------
//	Debug CLI

TT_CLI<CLI_MAX_INPUT_STR_LEN, CLI_MAX_HISTORY_STR>
	DBG_CLI_USB(&DBG_CLI_IOStruct, INVITE_STRING, CLI_CommandsParser);

TT_CLI<CLI_MAX_INPUT_STR_LEN, CLI_MAX_HISTORY_STR>
	DBG_CLI_UART(&DBG_CLI_IOStruct, INVITE_STRING, CLI_CommandsParser);

//#####################################################################################################
//	TASKS
//	����� ��������� �������� ��������� CLI. �������������� ������ CLI � ��������� ��������

//-----------------------------------------------------------------------------------------------------
//	USB

extern "C" void DBG_CLI_USB_Task()
{
	while (1)
	{
		DBG_CLI_USB.Process();
	}
}

// Serial
extern "C" void DBG_CLI_Serial_Task()
{
	while (1)
	{
		DBG_CLI_UART.Process();
	}
}