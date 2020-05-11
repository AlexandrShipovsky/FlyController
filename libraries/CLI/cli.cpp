/*
 * cli.cpp
 *
 *  Created on: 12 ˜˜˜˜. 2019 ˜.
 *      Author: d.semenyuk (injen-d)
 */

#include <stdlib.h>
#include <string.h>
#include "cli_base.h"


//˜˜˜˜˜˜˜ ˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜ strcmp() ˜ ˜˜˜, ˜˜˜ cmd_cmp() ˜˜˜˜˜˜˜˜˜˜ false ˜˜˜˜˜ ˜˜ ˜˜˜ ˜˜˜˜˜˜ ˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜˜˜
extern "C" bool cli_cmd_cmp(const char * str1, const char * str2)
{
	while(1)
	{
		if(*str1 != *str2) return false;
		if(*str1 == '\0') return true;
		str1++;
		str2++;
	}
}

// ˜˜˜˜˜˜˜˜ ˜˜˜ ˜˜˜˜˜ ˜ ˜˜˜˜˜˜ ˜˜ ˜˜˜˜˜˜˜˜˜
extern "C" void cli_strupr(char *s)
{
	do{
		if(*s >= 'a' && *s <= 'z') {*s = *s - 0x20;}
		s++;
	}while(*s != '\0');
}

// ˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜ ˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜ ˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜
// ˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜ ˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜, ˜˜ ˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜ '\0'
extern "C" bool cli_prepare_str(char* const ps, CLI_InputStrLen_t* const len)
{
	*len = strlen(ps);
	if(*len == 0) {return false;}
	cli_strupr(ps);
	for(CLI_InputStrLen_t i = 0; i < *len; i++)
	{
		register char c = ps[i];
		if(c==0x20 || c=='\t' || c==',' || c=='.' || c=='=' || c==';' || c==':' || c==0xd || c==0xa) {ps[i] = '\0';}
	}
	return true;
}

// ˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ps ˜˜ ˜˜˜˜˜˜ ˜˜˜˜˜ ˜ ˜˜˜˜˜˜˜, ˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜ len
extern "C" bool cli_seek_first_word(char**const ps, CLI_InputStrLen_t* const len)
//extern "C" bool cli_seek_first_word(char* &ps, CLI_InputStrLen_t &len)
{
	while(1)
	{
		if(*len == 0)	{return false;}
		if(**ps != '\0')	{return true;}
		(*ps)++;
		(*len)--;
	}
}

// ˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ps ˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜ ˜ ˜˜˜˜˜˜˜, ˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜ len
extern "C" bool cli_seek_next_word(char**const ps, CLI_InputStrLen_t* const len)
//extern "C" bool cli_seek_next_word(char* &ps, CLI_InputStrLen_t &len)
{
	while(1)
	{
		if(*len == 0)	{return false;}
		if(**ps == '\0')	{break;}
		(*ps)++;
		(*len)--;
	}
	return cli_seek_first_word(ps, len);
}


