/*
 * cli_base.h
 */

#ifndef APP_CLI_CLI_BASE_H_
#define APP_CLI_CLI_BASE_H_

#include <stdint.h>
#include <string.h>
#include "cli.h"
#include "cli_config.h"
#include "cli_port.h"

namespace cli{

typedef void (*cli_UserCommandsParser_t)(const TCLI_IO* const io, char* ps, CLI_InputStrLen_t len);

void cli_sleep(uint32_t time_ms);

//#####################################################################################################
//	CLI Plugins

//------------------------------------------------------------------------------------------------------------------------------
// ˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜
template<int8_t max_history_strings,  CLI_InputStrLen_t max_history_string_len>
class TT_CLI_History
{
	int8_t seek_history_wr;
	int8_t seek_history_rd;
	char cmd_history[max_history_strings][max_history_string_len];

  public:

	TT_CLI_History():
		seek_history_wr(0),
		seek_history_rd(0)
	{
		for(int8_t i = 0; i < max_history_strings; i++) {cmd_history[i][0] = '\0';}
	}

	void AddToHistory(const char * const s)
	{
		if(strlen(s) == 0) {return;}
		for(int8_t i = 0; i < max_history_strings; i++)
		{
			if(cli_cmd_cmp(s, cmd_history[i]))
			{
				seek_history_rd = i;
				return;
			}
		}
		strcpy(cmd_history[seek_history_wr], s);
		seek_history_rd = seek_history_wr;
		if (++seek_history_wr >= max_history_strings) {seek_history_wr = 0;}
	}

	CLI_InputStrLen_t ReadFromHistoryBottom(char* const s)
	{
		for(int8_t i = 0; i < max_history_strings; i++)
		{
			if(++seek_history_rd >= max_history_strings) {seek_history_rd = 0;}
			CLI_InputStrLen_t len = strlen(cmd_history[seek_history_rd]);
			if(len > 0)
			{
				strcpy(s, cmd_history[seek_history_rd]);
				return len;
			}
		}
		s[0] = 0;
		return 0;
	}

	CLI_InputStrLen_t ReadFromHistoryTop(char* const s)
	{
		for(int8_t i = 0; i < max_history_strings; i++)
		{
			CLI_InputStrLen_t len = strlen(cmd_history[seek_history_rd]);
			if(len > 0)
			{
				strcpy(s,cmd_history[seek_history_rd]);
				if(--seek_history_rd < 0) {seek_history_rd = max_history_strings-1;}
				return len;
			}
			if(--seek_history_rd < 0) {seek_history_rd = max_history_strings-1;}
		}
		s[0] = 0;
		return 0;
	}
};

//------------------------------------------------------------------------------------------------------------------------------
// ˜˜˜˜˜˜˜˜˜˜ "˜˜˜˜˜˜˜ ˜˜˜˜˜˜"
template <CLI_InputStrLen_t cmd_max_len>
class TT_CLI_Binder
{
  public:
	enum FuncKeys_t{KeyF1=0, KeyF2, KeyF3, KeyF4, KeyF5, KeyF6, KeyF7, KeyF8, KeyF9, KeyF10, KeyF11, KeyF12, NumberOfKeys};
  protected:
	struct TKeyCmdBinder{
		const char* const key_name;
		char cmd[cmd_max_len];
	};
	TKeyCmdBinder KeyCmdBinder[NumberOfKeys] = {{"F1" }, {"F2" }, {"F3" }, {"F4" }, {"F5" }, {"F6" }, {"F7" }, {"F8" }, {"F9" }, {"F10" }, {"F11" }, {"F12" }};

	const TCLI_IO* const io;

  public:
	TT_CLI_Binder(const TCLI_IO* const io) :
		io(io)
	{
		for(int i=0; i<NumberOfKeys; i++)
		{
			KeyCmdBinder[i].cmd[0] = '\0';
		}
	}
	CLI_InputStrLen_t ReadFromBinder(FuncKeys_t key, char* const cli_cmd_string)
	{
		if(KeyCmdBinder[key].cmd[0] == '\0') {return 0;}
		for(CLI_InputStrLen_t i = 0; i < cmd_max_len; i++)
		{
			cli_cmd_string[i] = KeyCmdBinder[key].cmd[i];
			if(KeyCmdBinder[key].cmd[i] == '\0') {return i;}
		}
		KeyCmdBinder[key].cmd[0] = '\0';
		return 0;
	}

	void bind(const char* const key_name, const char* const command, CLI_InputStrLen_t cmd_len)
	{
		for(int i=0; i<NumberOfKeys; i++)
		{
			if(cli_cmd_cmp(key_name, KeyCmdBinder[i].key_name))
			{
				for(CLI_InputStrLen_t n = 0; n < cmd_len; n++)
				{
					KeyCmdBinder[i].cmd[n] = (command[n] == '\0') ? ' ' : command[n];
				}
				KeyCmdBinder[i].cmd[cmd_len] = '\0';
				io->printf("On key %s binded command: %s\n", KeyCmdBinder[i].key_name, KeyCmdBinder[i].cmd);
				return;
			}
		}
		io->printf(CLI_STR_INVALID_KEYWORD);
	}

	void unbind(const char* const key_name)
	{
		bool all_keys_flag = false;
		if(cli_cmd_cmp(key_name, "ALL")) {all_keys_flag = true;}
		for(int i=0; i<NumberOfKeys; i++)
		{
			if(all_keys_flag || cli_cmd_cmp(key_name, KeyCmdBinder[i].key_name))
			{
				KeyCmdBinder[i].cmd[0] = '\0';
				if(!all_keys_flag)
				{
					io->printf("Key %s - unbinded!\n", KeyCmdBinder[i].key_name);
					return;
				}
			}
		}
		if(all_keys_flag) {io->printf("All keys unbinded!\n");}
		else {io->printf(CLI_STR_INVALID_KEYWORD);}
	}
};

//------------------------------------------------------------------------------------------------------------------------------
// ˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜
template <CLI_InputStrLen_t cmd_max_len>
class TT_CLI_CycleCmd
{
	const TCLI_IO* const io;
	volatile uint32_t Rpt;
	uint32_t Pause;
  public:
	char cmd[cmd_max_len];

	TT_CLI_CycleCmd(const TCLI_IO* const io) :
		io(io),
		Rpt(0),
		Pause(0)
	{}

	void cycle_cmd_init(const char* const cmd_string, uint32_t cmd_len, uint32_t repeats, uint32_t pause_ms)
	{
		if(Rpt > 0) {return;}
		for(uint32_t n = 0; n < cmd_len; n++) {	cmd[n] = (cmd_string[n] == '\0') ? ' ' : cmd_string[n]; }
		cmd[cmd_len] = '\0';
		Pause = pause_ms;
		Rpt = repeats;
	}

	//˜˜˜˜˜˜˜˜˜˜ true ˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜, ˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜ ˜ ˜˜˜˜˜
	CLI_InputStrLen_t cycle_cmd(char* const cli_cmd_string, const char break_char)
	{
		static bool not_first_repeat = false;
		uint32_t rpt_tmp = Rpt;
		if(rpt_tmp > 0)
		{
			if(not_first_repeat)
			{
				uint32_t ms_counter = 0;
				while(1)
				{
					char c;
					if(io->read_char(&c, 0))
					{
						if(c == break_char)	//˜˜˜˜˜˜˜ ˜˜˜˜˜˜ ˜˜˜˜˜˜
						{
							Rpt = 0;
							not_first_repeat = false;
							return 0;
						}
					}
					if(ms_counter == Pause)
					{
						break;
					}
					cli_sleep(CLI_TICK_DELAY_1ms);
					ms_counter++;
				}
			}
			not_first_repeat = true;
			rpt_tmp--;
			Rpt = rpt_tmp;
			//˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜ ˜ ˜˜˜˜˜
			strcpy(cli_cmd_string, cmd);
			return strlen(cli_cmd_string);
		}
		not_first_repeat = false;
		return 0;
	}
};

//#####################################################################################################
//	CLI Template Class

template
<
		CLI_InputStrLen_t cmd_max_len,
		int8_t max_history_strings
>
class TT_CLI :
		private TT_CLI_History<max_history_strings, cmd_max_len>,
		private TT_CLI_Binder<cmd_max_len>,
		private TT_CLI_CycleCmd<cmd_max_len>
{
	typedef TT_CLI_History<max_history_strings, cmd_max_len> TT_CLI_History_t;
	typedef TT_CLI_Binder<cmd_max_len> TT_CLI_Binder_t;
	typedef TT_CLI_CycleCmd<cmd_max_len> TT_CLI_CycleCmd_t;
private:
	char cmd_buf[cmd_max_len];
public:
	const TCLI_IO* const io;
	const char* const invite;
private:
	cli_UserCommandsParser_t UserParser;
public:

	TT_CLI(const TCLI_IO* const io, const char* invite_p, cli_UserCommandsParser_t UserParser):
		invite(invite_p),
		io(io),
		UserParser(UserParser),
		TT_CLI_History<max_history_strings, cmd_max_len>(),
		TT_CLI_Binder<cmd_max_len>(io),
		TT_CLI_CycleCmd<cmd_max_len>(io)
	{}

	CLI_InputStrLen_t ReadString(char* const s)				//˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜˜˜ ˜ ˜˜˜˜˜˜˜˜˜˜ ˜ ˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜
	{
		CLI_InputStrLen_t length = 0;
		CLI_InputStrLen_t cursor = 0;
		io->printf("\r%s", invite);
		while(1)
		{
			char c;
			if(!io->read_char(&c, CLI_MAX_DELAY)) {return 0;}
			if(c == '\x1b') // ESC (˜˜˜˜˜˜ ˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜)
			{
				if(!io->read_char(&c, 100))
				{//˜˜˜˜ ˜˜ ˜˜˜˜˜˜ ˜˜˜˜˜˜ ˜˜˜˜˜˜ - ˜˜˜˜˜˜˜˜ ˜˜˜˜ ˜˜˜˜˜˜ ˜˜˜˜˜˜˜ Esc

					continue;
				}
				switch(c)
				{
				case '[':	//CSI-˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜ "\x1b["
				{
					if(!io->read_char(&c, CLI_MAX_DELAY)) {return 0;}
					switch(c)
					{
	//UP
					case 'A': //UP
						length = TT_CLI_History_t::ReadFromHistoryTop(s);
						cursor = length;
						io->printf("\x1b[2K\r");		//˜˜˜˜˜˜˜ ˜˜˜ ˜˜˜˜˜˜
						io->printf("\r%s%s", invite, s);
						break;
	//DOWN
					case 'B': //DOWN
						length = TT_CLI_History_t::ReadFromHistoryBottom(s);
						cursor = length;
						io->printf("\x1b[2K\r"); 	//˜˜˜˜˜˜˜ ˜˜˜ ˜˜˜˜˜˜
						io->printf("\r%s%s", invite, s);
						break;
	//RIGHT
					case 'C': //RIGHT
						if(cursor < length)
						{
							cursor++;
							io->printf("\x1b[C");	//˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜ 1 ˜˜˜˜˜˜˜ ˜˜˜˜˜˜
						}
						break;
	//LEFT
					case 'D': //LEFT
						if(cursor)
						{
							cursor--;
							io->printf("\x1b[D"); 	//˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜ 1 ˜˜˜˜˜˜˜ ˜˜˜˜˜
						}
						break;
	//HOME
					case 'H': //HOME
						if(cursor) {io->printf("\x1b[%uD", cursor);}  //˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜ ˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜
						cursor=0;
						break;
					case '1': //F5-F8, HOME
					{
						if(!io->read_char(&c, CLI_MAX_DELAY))  {break;}
						if(c == '~') //HOME
						{
							if(cursor) {io->printf("\x1b[%uD", cursor);}  //˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜ ˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜
							cursor=0;
							break;
						}
						char c2;
						if(!io->read_char(&c2, CLI_MAX_DELAY)) {break;}
						if(c2 != '~') {break;}
						typename TT_CLI_Binder_t::FuncKeys_t indx = TT_CLI_Binder_t::NumberOfKeys;
						switch(c)
						{
						case '1': //F1
						{
							indx = TT_CLI_Binder_t::KeyF1;
							break;
						}
						case '2': //F2
						{
							indx = TT_CLI_Binder_t::KeyF2;
							break;
						}
						case '3': //F3
						{
							indx = TT_CLI_Binder_t::KeyF3;
							break;
						}
						case '4': //F4
						{
							indx = TT_CLI_Binder_t::KeyF4;
							break;
						}
						case '5': //F5
						{
							indx = TT_CLI_Binder_t::KeyF5;
							break;
						}
						case '7': //F6
						{
							indx = TT_CLI_Binder_t::KeyF6;
							break;
						}
						case '8': //F7
						{
							indx = TT_CLI_Binder_t::KeyF7;
							break;
						}
						case '9': //F8
						{
							indx = TT_CLI_Binder_t::KeyF8;
							break;
						}
						}
						if(indx >= TT_CLI_Binder_t::NumberOfKeys) {break;}
						length = TT_CLI_Binder_t::ReadFromBinder(indx, s);
						if(length > 0)
						{
							io->printf("\r%s%s\n", invite, s);
							return length;
						}
						break;
					}
					case '2': //F9-F12
					{
						if(!io->read_char(&c, CLI_MAX_DELAY))  {break;}
						char c2;
						if(!io->read_char(&c2, CLI_MAX_DELAY))  {break;}
						if(c2 != '~')  {break;}
						typename TT_CLI_Binder_t::FuncKeys_t indx = TT_CLI_Binder_t::NumberOfKeys;
						switch(c)
						{
						case '0': //F9
						{
							indx = TT_CLI_Binder_t::KeyF9;
							break;
						}
						case '1': //F10
						{
							indx = TT_CLI_Binder_t::KeyF10;
							break;
						}
						case '3': //F11
						{
							indx = TT_CLI_Binder_t::KeyF11;
							break;
						}
						case '4': //F12
						{
							indx = TT_CLI_Binder_t::KeyF12;
							break;
						}
						}
						if(indx >= TT_CLI_Binder_t::NumberOfKeys) {break;}
						length = TT_CLI_Binder_t::ReadFromBinder(indx, s);
						if(length > 0)
						{
							io->printf("\r%s%s\n", invite, s);
							return length;
						}
						break;
					}
	//END
					case '4': //END
						if(!io->read_char(&c, CLI_MAX_DELAY)) {return 0;}
						if(c != '~') {break;}
					case 'F': //END
						if(cursor < length)
						{
							if(length - cursor) {io->printf("\x1b[%uC", length-cursor);} //˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜ ˜˜˜˜˜ ˜˜˜˜˜˜
							cursor = length;
						}
						break;
	//DELETE #1
					case '3': //DELETE
						if(!io->read_char(&c, CLI_MAX_DELAY)) {return 0;}
						if(c=='~')
						{
							if(cursor < length)
							{
								for(unsigned int i = cursor; i < length; i++) {s[i] = s[i+1];}	//˜˜˜˜˜˜ ˜˜˜˜˜˜ ˜˜ ˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜ ˜˜ 1 ˜˜˜˜˜˜˜ ˜˜˜˜˜
								length--;
								io->printf("%s \b", &s[cursor]);
								if(length - cursor) {io->printf("\x1b[%uD", length-cursor);}		//˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜ ˜˜˜˜˜ ˜˜˜˜˜
							}
						}
						break;
					}//switch(c)
					break;
				}
				case 'O':	//SS3-˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜ "\x1bO"
				{
					if(!io->read_char(&c, CLI_MAX_DELAY)) {return 0;}
					switch(c)
					{
	//CTRL+UP
					case 'A': //CTRL+UP
					{

						break;
					}
	//CTRL+DOWN
					case 'B': //CTRL+DOWN
					{

						break;
					}
	//CTRL+RIGHT
					case 'C': //CTRL+RIGHT
					{

						break;
					}
	//CTRL+LEFT
					case 'D': //CTRL+LEFT
					{

						break;
					}
	//F1
					case 'P': //F1
					{
						unsigned int len = TT_CLI_Binder_t::ReadFromBinder(TT_CLI_Binder_t::KeyF1, s);
						if(len > 0)
						{
							io->printf("\r%s%s\n", invite, s);
							return length = len;
						}
						break;
					}
	//F2
					case 'Q': //F2
					{
						unsigned int len = TT_CLI_Binder_t::ReadFromBinder(TT_CLI_Binder_t::KeyF2, s);
						if(len > 0)
						{
							io->printf("\r%s%s\n", invite, s);
							return length = len;
						}
						break;
					}
	//F3
					case 'R': //F3
					{
						unsigned int len = TT_CLI_Binder_t::ReadFromBinder(TT_CLI_Binder_t::KeyF3, s);
						if(len > 0)
						{
							io->printf("\r%s%s\n", invite, s);
							return length = len;
						}
						break;
					}
	//F4
					case 'S': //F4
					{
						unsigned int len = TT_CLI_Binder_t::ReadFromBinder(TT_CLI_Binder_t::KeyF4, s);
						if(len > 0)
						{
							io->printf("\r%s%s\n", invite, s);
							return length = len;
						}
						break;
					}
					}//switch(c)
					break;
				}
				}//switch(c)
			}//if(c=='\x1b')
			else
			{
				switch(c)
				{
	//ENTER
				case '\r': //0xd:  //ENTER: "\r\n"
				case '\n': //0xa:
				{
					s[length] = 0;
					io->printf("\r%s%s\n", invite, s);
					TT_CLI_History_t::AddToHistory(s);
					return length;
				}
	//BACKSPASE
				case '\b': //BACKSPASE
				{
					if(length == cursor)
					{
						if(length)
						{
							length--;
							cursor = length;
							s[length] = '\0';
							io->printf("\b \b");
						}
					}
					else if(cursor < length)
					{
						if(cursor)
						{
							for(CLI_InputStrLen_t i = cursor; i <= length; i++) {s[i-1] = s[i];} //˜˜˜˜˜˜ ˜˜˜˜˜˜ ˜˜ ˜˜˜˜˜˜˜ (˜˜˜˜˜˜˜ ˜˜˜˜˜˜) ˜˜˜˜˜˜˜˜ ˜˜ 1 ˜˜˜˜˜˜˜ ˜˜˜˜˜
							cursor--;
							length--;
							io->printf("\b%s \b", &s[cursor]);
							if(length-cursor) {io->printf("\x1b[%uD", length-cursor);} 		//˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜ ˜˜˜˜˜ ˜˜˜˜˜
						}
					}
					break;
				}
	//DELETE #2
				case 0x7f: //DELETE
				{
					if(cursor < length)
					{
						for(CLI_InputStrLen_t i = cursor; i < length; i++) {s[i] = s[i+1];}		//˜˜˜˜˜˜ ˜˜˜˜˜˜ ˜˜ ˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜ ˜˜ 1 ˜˜˜˜˜˜˜ ˜˜˜˜˜
						length--;
						io->printf("%s \b", &s[cursor]);
						if(length-cursor) {io->printf("\x1b[%uD", length-cursor);}			//˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜ ˜˜˜˜˜ ˜˜˜˜˜
					}
					break;
				}
	//OTHER KEYS
				default:
				{
					if(c >= 0x20) // ' '
					{
						if(length < (CLI_MAX_INPUT_STR_LEN-2))
						{
							if(length == cursor)
							{
								s[length++] = c;
								cursor = length;
								s[length] = '\0';
								io->putchar(c);
							}
							else if(cursor < length)
							{
								for(CLI_InputStrLen_t i = length; i >= cursor; i--) {s[i+1] = s[i];}	//˜˜˜˜˜˜ ˜˜˜˜˜˜ ˜˜ ˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜ ˜˜ 1 ˜˜˜˜˜˜˜ ˜˜˜˜˜˜
								s[cursor] = c;
								io->printf("%s", &s[cursor]);
								cursor++;
								length++;
								if(length-cursor) {io->printf("\x1b[%uD", length-cursor);}		//˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜ ˜˜˜˜˜ ˜˜˜˜˜
							}
						}
					}
					break;
				}
				}//switch(c)
			}//if(c=='\x1b') else
		}//while(1)
	}

	//˜˜˜˜˜˜˜˜˜˜˜˜ ˜ ˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜, ˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜, ˜ ˜˜˜˜˜ ˜˜˜˜˜˜˜˜ UserCommands()
	void Pars(char* ps, CLI_InputStrLen_t len)
	{
		if(!cli_prepare_str(ps, &len)){return;}
		if(!cli_seek_first_word(&ps, &len)){return;}

		//----------------------------------------------------------------------------------
		if(cli_cmd_cmp(ps,"HELP") || cli_cmd_cmp(ps,"?"))
		{
			io->printf("\r%s",CLI_HLP);
			strcpy(ps, "?");
		}

		//##################################################################################
		//xxx CLI ˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜
                /*
		//----------------------------------------------------------------------------------
		CLI_IF_CMD("SAVECFG", "˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜ ˜˜˜")
		{
			CLI_NEXT_WORD();
			TypeSavedSettings_t TypeSavedSettings = TypeSavedSettings_invalid_val;
			CLI_IF_CMD("MAIN", "˜˜˜˜˜˜˜˜") {TypeSavedSettings = TypeSavedSettings_main;}
			CLI_IF_CMD("RSRV", "˜˜˜˜˜˜˜˜˜") {TypeSavedSettings = TypeSavedSettings_alt1;}
			if(TypeSavedSettings != TypeSavedSettings_invalid_val)
			{
				uint32_t status = SaveSettings(TypeSavedSettings);
				if(status != 0) {io->printf("\x1b[0;31mStore settings - failed!\nError code: %u\x1b[0m\n", status);}
				else {io->printf("\x1b[0;32mStore settings - success!\x1b[0m\n");}
				return;
			}
			CLI_INVALID_KEYWORD();
			return;
		}
		//----------------------------------------------------------------------------------
		CLI_IF_CMD("LOADCFG", "˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜ ˜˜˜")
		{
			CLI_NEXT_WORD();
			TypeSavedSettings_t TypeSavedSettings = TypeSavedSettings_invalid_val;
			CLI_IF_CMD("MAIN", "˜˜˜˜˜˜˜˜") {TypeSavedSettings = TypeSavedSettings_main;}
			CLI_IF_CMD("RSRV", "˜˜˜˜˜˜˜˜˜") {TypeSavedSettings = TypeSavedSettings_alt1;}
			if(TypeSavedSettings != TypeSavedSettings_invalid_val)
			{
				uint32_t status = LoadSettings(TypeSavedSettings);
				if(status != 0) {io->printf("\x1b[0;31mLoad settings - failed!\nError code: %u\x1b[0m\n", status);}
				else {io->printf("\x1b[0;32mLoad settings - success!\x1b[0m\n");}
				return;
			}
			CLI_INVALID_KEYWORD();
			return;
		}
		//----------------------------------------------------------------------------------
		CLI_IF_CMD("DEFCFG", "˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜-˜˜˜˜˜˜˜˜˜")
		{
			LoadDefaultSettings();
			io->printf("Default settings is loaded!\n");
			return;
		}
		//----------------------------------------------------------------------------------
		CLI_IF_CMD("GETCFG", "˜˜˜˜˜ ˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜")
		{
			PrintSettings(io->printf);
			ClbrPrint(io->printf);
			return;
		}*/
		//----------------------------------------------------------------------------------
		CLI_IF_CMD("CYCLE", "˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜, ˜˜˜˜˜˜: cycle 10 50 get param1\n\t (˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜ \"get param1\" 10 ˜˜˜ ˜ ˜˜˜˜˜˜ 50 ˜˜)")
		{
			uint32_t val1;
			CLI_NEXT_WORD();
			sscanf(ps, "%u", &val1);
			if(val1 == 0) {io->printf(CLI_STR_INVALID_KEYWORD); return;}
			uint32_t val2;
			CLI_NEXT_WORD();
			sscanf(ps, "%u", &val2);
			CLI_NEXT_WORD();
			TT_CLI_CycleCmd<cmd_max_len>::cycle_cmd_init(ps, len, val1, val2);
			io->printf("˜ommand \"%s\" repeat %u times with pause %u ms ...\n", TT_CLI_CycleCmd<cmd_max_len>::cmd, val1, val2);
			return;
		}
		//----------------------------------------------------------------------------------
		CLI_IF_CMD("BIND", "˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜ (F1...F12), ˜˜˜˜˜˜: bind f1 set param1 123")
		{
			CLI_NEXT_WORD();
			char* key_name = ps;
			CLI_NEXT_WORD();
			TT_CLI_Binder<cmd_max_len>::bind(key_name, ps, len);
			return;
		}
		//----------------------------------------------------------------------------------
		CLI_IF_CMD("UNBIND", "˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜ (F1...F12, all - ˜˜˜ ˜˜˜˜˜), ˜˜˜˜˜˜: unbind f1")
		{
			CLI_NEXT_WORD();
			TT_CLI_Binder<cmd_max_len>::unbind(ps);
			return;
		}
		//----------------------------------------------------------------------------------
		//##################################################################################
		UserParser(io, ps, len);	//˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜
	}

	void Process()
	{
		CLI_InputStrLen_t len = TT_CLI_CycleCmd_t::cycle_cmd(cmd_buf, ' ');
		if(len == 0)
		{
			len = ReadString(cmd_buf);
			if(len == 0) {return;}
		}
	 	Pars(cmd_buf, len);
	}
};

}//namespace cli

#endif /* APP_CLI_CLI_BASE_H_ */
