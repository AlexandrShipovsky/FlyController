/*
 * cli_cmd.h
 */

#ifndef APP_CLI_CMD_H_
#define APP_CLI_CMD_H_

#include "cli_base.h"


extern "C" void CLI_CommandsParser(const TCLI_IO* const io, char* ps, CLI_InputStrLen_t len);

#define ok DbgPrintf("\x1b[0;32mOk\n\r\x1b[0m")



#endif /* APP_CLI_CMD_H_ */

