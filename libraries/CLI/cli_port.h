/*
 * cli_port.h
 *
 *  Created on: 12 ����. 2019 �.
 *      Author: d.semenyuk
 */

#ifndef APP_CLI_CLI_PORT_H_
#define APP_CLI_CLI_PORT_H_

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


#define CLI_MAX_DELAY		(-1)
#define CLI_TICK_DELAY_1ms	1



//#	define NO_INIT __attribute__((section(".noinit")))
//#	define STDARG_VALIST __VALIST
//#	ifndef NORETURN
//#		define NORETURN    __attribute__((__noreturn__))
//#	endif


//enum ComPortsName_t{UART1, UART2};
//
//struct TSerialParam{
//	ComPortsName_t PortName;
//	uint32_t BaudRate;
//	uint16_t RxBufLen;
//	uint16_t TxBufLen;
//};



#endif /* APP_CLI_CLI_PORT_H_ */
