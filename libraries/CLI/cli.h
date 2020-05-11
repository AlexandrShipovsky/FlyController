/*
 * cli.h
 *
 *  Created on: 12 ����. 2019 �.
 *      Author: d.semenyuk
 */

#ifndef APP_CLI_H_
#define APP_CLI_H_

#include <stdint.h>
#include "cli_config.h"
#include "cli_port.h"

static const char CLI_HLP[]={
		"\n�������\n"
		"\n"
		"������� ������ ������:\n"
		" ... - �� ����� ���������� ������ ���� ���� �� ������������� ���� ������.\n"
		" (w, u, x, y, u, z) - ����� ���� ������ ���� �� ������������� �\n\t ������� ������� ����� ������� ���������� ��� ������.\n"
		" {x} - � �������� ������� ����������� �������������� ���������.\n"
		" [x..X] - � ������������� ������� ����������� �������� ���������� �������� ��� �����,\n\t �������������� � ������� �� ���� �����.\n"
		"\n�������:\n"
};
static const char CLI_STR_UNKNOWN_COMMAND[] = "\r\x1b[0;31mUnknown command!\x1b[0m\n";
//const char STR_UNKNOWN_COMMAND[] = "\r\x1b[1;31m������������ ������� ��� ��������!\x1b[0m\n";
static const char CLI_STR_INVALID_KEYWORD[] = "\r\x1b[0;31mInvalid keyword!\x1b[0m\n";
static const char CLI_STR_INVALID_VALUE[] = "\r\x1b[0;31mInvalid value!\x1b[0m\n";
//static const char CLI_STR_INVALID_KEYWORD[] = "\a\r\x1b[0;31mInvalid keyword!\x1b[0m\n";
//const char STR_INVALID_KEYWORD[] = "\a\r\x1b[1;31m����������� �������!\x1b[0m\n";


#define CLI_IF_CMD(Command, HelpString) \
	if(cli_cmd_cmp(ps, "?")){io->printf(" %s: %s\n", Command, HelpString);}\
	else if(cli_cmd_cmp(ps, Command))

#define CLI_SCAN_PARAM(Format, Variable, HintString) \
	if(cli_seek_next_word(&ps, &len)){ sscanf(ps, Format, &Variable); } \
	else { io->printf(HintString); io->scanf(Format, &Variable); io->printf("\n"); }

#define CLI_NEXT_WORD() if( *ps != '?' ){if(!cli_seek_next_word(&ps, &len)){io->printf(CLI_STR_INVALID_KEYWORD); return;}}

#define CLI_UNKNOWN_COMMAND() if( *ps != '?' ){io->printf(CLI_STR_UNKNOWN_COMMAND);}
#define CLI_INVALID_KEYWORD() if( *ps != '?' ){io->printf(CLI_STR_INVALID_KEYWORD);}

#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

//#####################################################################################################

typedef struct{
	void (*putchar)		(char val);
	bool (*read_char)	(char* const val, uint32_t timeout);
	int  (*printf)		(const char *format, ...);
	int  (*scanf)		(const char *format, ...);
} TCLI_IO;

//#####################################################################################################
//	CLI Common functions

//������� �� ������������ strcmp() � ���, ��� cmd_cmp() ���������� false ����� �� ��� ������ ����� ���������� ��������������
bool cli_cmd_cmp(const char * str1, const char * str2);

// �������� ��� ����� � ������ �� ���������
void cli_strupr(char *s);

// ��������� ����� �� ��������� ������� �� ��������� �������
// ���������� ����� ���������� �������� �� ��������� �������, �� ������ ���������� ������� '\0'
bool cli_prepare_str(char* const ps, CLI_InputStrLen_t* const len);

// ��������� ��������� ps �� ������ ����� � �������, ������������ ����� len
bool cli_seek_first_word(char**const ps, CLI_InputStrLen_t* const len);

// ��������� ��������� ps �� ��������� ����� � �������, ������������ ����� len
bool cli_seek_next_word(char**const ps, CLI_InputStrLen_t* const len);

//#####################################################################################################
//#####################################################################################################
//

int DbgPrintf(const char *format, ...);

void DBG_CLI_Serial_Task();
void DBG_CLI_USB_Task();
void CLI_Init(void);
#ifdef __cplusplus
}
#endif


#define cli_printf DbgPrintf


#endif /* APP_CLI_H_ */

