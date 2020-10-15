/*
 * cli_config.h
 *
 */

#ifndef APP_CLI_CLI_CONFIG_H_
#define APP_CLI_CLI_CONFIG_H_

#include <stdint.h>
#include <stdio.h>

#define CLI_MAX_INPUT_STR_LEN	64		//������������ ����� �������� ������

#define CLI_MAX_HISTORY_STR		8		//������������ ���������� �������� � ������� ������


typedef int CLI_InputStrLen_t;		//��� ���������� ��� �������� ����� ��������� ������ (������ ���� ��������)


#define INVITE_STRING "\x1b[0;35mFlyController>\x1b[0m"


#define CLI_PRINTF_WORK_BUFFER_SIZE	(1024)
#define CLI_SCANF_WORK_BUFFER_SIZE	(CLI_MAX_INPUT_STR_LEN)

#endif /* APP_CLI_CLI_CONFIG_H_ */
