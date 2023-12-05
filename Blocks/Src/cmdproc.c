/*
	gbm blocks, h/w independent
	stack-based command processor core and std commands
	gbm, 10'2018
*/

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "cmdproc.h"

_Bool echo_on = 1;
uint8_t cmdline_start;	// cursor x after prompt displayed

//==============================================================================
// UTIL
//==============================================================================
//static inline uint8_t hex_digit(uint32_t val)
//{
//	return "0123456789abcdef"[val & 0xf];
//}

//==============================================================================
// STACK
//==============================================================================
uint32_t stack[STACK_SIZE];
uint8_t	stack_idx = 0;
char argstring[ARGSTRLEN + 1];

#define STACK_TOP	(stack[stack_idx])

uint32_t stack_get(uint8_t idx)
{
	return stack[(stack_idx + STACK_SIZE - idx) % STACK_SIZE];
}

void stack_push(uint32_t v)
{
	stack_idx = (stack_idx + 1) % STACK_SIZE;
	stack[stack_idx] = v;
}

uint32_t stack_pop(void)
{
	uint32_t v = stack[stack_idx];
	stack_idx = (stack_idx + STACK_SIZE - 1) % STACK_SIZE;
	return v;
}

//==============================================================================
// COMMANDS
//==============================================================================
// help and utilities

static uint8_t cmd_and(void)
{
	uint32_t a = stack_pop();
	
	STACK_TOP &= a;
	return 0;
}

static uint8_t cmd_mul(void)
{
	uint32_t a = stack_pop();
	
	STACK_TOP *= a;
	return 0;
}

static uint8_t cmd_plus(void)
{
	uint32_t a = stack_pop();
	
	STACK_TOP += a;
	return 0;
}

static uint8_t cmd_plusplus(void)
{
	STACK_TOP++;
	return 0;
}

static uint8_t cmd_minus(void)
{
	uint32_t a = stack_pop();
	
	STACK_TOP -= a;
	return 0;
}

static uint8_t cmd_minusminus(void)
{
	STACK_TOP--;
	return 0;
}

static uint8_t print_stack(const char *fmt)
{
	char resp[11];
	
	for (uint8_t i = STACK_SIZE; i > 0;)
	{
		sprintf(resp, fmt, stack_get(--i));
		con_putstr(resp);
	}
	con_put_nl();
		
	return 0;
}

static uint8_t cmd_dotdot(void)
{
	return print_stack(" %" PRIu32);
}

static uint8_t cmd_dotdotx(void)
{
	return print_stack("0x%" PRIx32);
}

static uint8_t cmd_div(void)
{
	uint32_t a = stack_pop();
	
	STACK_TOP /= a;
	return 0;
}

static uint8_t cmd_xor(void)
{
	uint32_t a = stack_pop();
	
	STACK_TOP ^= a;
	return 0;
}

static uint8_t cmd_readword(void)
{
	uint32_t a = stack_pop();
	
	a = *(uint32_t *)a;
	char resp[11];
	sprintf(resp, "%08" PRIx32 "\r\n", a);
	con_putstr(resp);
	return 0;
}

static uint8_t cmd_or(void)
{
	uint32_t a = stack_pop();
	
	STACK_TOP |= a;
	return 0;
}

static const struct cmd_ stdcmdtab[] = {
	{"&", 		cmd_and},		// and
	{"*", 		cmd_mul},		// 
	{"+", 		cmd_plus},		// add
	{"++", 		cmd_plusplus},	// increment
	{"-", 		cmd_minus},		// sub
	{"--", 		cmd_minusminus},	// decrement
	{"..", 		cmd_dotdot},	// display stack content in decimal
	{"..x", 	cmd_dotdotx},	// display stack content in hex
	{"/", 		cmd_div},		// 
	{"^", 		cmd_xor},		// 
//	{"rb",		cmd_readbyte},
	{"rw",		cmd_readword},
	{"|", 		cmd_or},		// or
	{0, 0}
};

_Bool cmd_tlookup(const char *cmds, const struct cmd_ *ctab)
{
	int8_t s;
	for (s = 1; ctab->str && (s = strcmp(cmds, ctab->str)) > 0; ctab ++);
	if (s)
		return 0;
	ctab->fun();
	return 1;
}

_Bool stdcmd_lookup(const char *cmds)
{
	return cmd_tlookup(cmds, stdcmdtab);
}
