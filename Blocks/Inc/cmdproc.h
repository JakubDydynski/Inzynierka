/*
Console command processor defs
gbm, 2017..2023
*/

#include "cmdi.h"	// application-specific stuff

#ifndef STACK_SIZE
#define STACK_SIZE	4u
#endif
#define ARGSTRLEN	32u

// defined in console interface module
void con_putchar(char c);

// defined in cmdedit.c
void con_putstr(const char *s);
void con_put_nl(void);

extern char argstring[ARGSTRLEN + 1];
extern uint32_t stack[STACK_SIZE];
extern uint8_t	stack_idx;
extern uint8_t cmdline_start;	// cursor x after prompt displayed

#define STACK_TOP	(stack[stack_idx])

uint32_t stack_get(uint8_t idx);
void stack_push(uint32_t v);
uint32_t stack_pop(void);

struct cmd_ {
	char *str;
	uint8_t (*fun)(void);
};

_Bool cmd_tlookup(const char *cmds, const struct cmd_ *ctab);
_Bool stdcmd_lookup(const char *cmds);
uint8_t cmd_interpret(const char* restrict s);
