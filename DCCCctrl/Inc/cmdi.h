/* 
	Appplication-specific console commands related definitions.
	Julia Kosowska 11'2016, gbm ..2023
	
	included by cmdproc.h
*/
	
#ifndef __CMDI_H
#define __CMDI_H

#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>

#define MAXCMDLINE	80u
#define NCMDS	6u
#define STACK_SIZE	8u

#define CMDI_REPEATONEMPTY

void con_putchar(char c);	// defined in xxx_console.c

extern _Bool echo_on;
extern _Bool hex_error;
extern volatile _Bool new_prompt;
extern uint8_t (*lastcmd)(void);	// command to repeat (if not null)
void cmd_repeat(void);

//extern volatile uint32_t time_cnt;
uint8_t prompt(void);
void cmdi_start(void);
uint8_t cmdi(char* s, uint32_t len);

// defined in the main command interpreter, used by console interface
void display_signon(void);
void display_prompt(void);

// defined in the main command interpreter, used by cmdproc.c
_Bool cmd_lookup(const char *cmds);
#endif
