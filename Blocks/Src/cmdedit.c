/*
	gbm blocks, h/w independent
	Console command input with history buffer using ANSI terminal.
	10'2018..2023
*/
#include <stdio.h>
#include <string.h>
#include "cmdproc.h"

void con_putstr(const char *s)
{
	while (*s)
		con_putchar(*s++);
}

void con_put_nl(void)
{
	con_putstr("\r\n");
}

// move cursor to x position within edited line (realtive to prompt)
static void cursor_set_x(uint8_t x)
{
	con_putstr("\x1B[");	// CHA
	char s[3];
	sprintf(s, "%d", cmdline_start + x);
	con_putstr(s);
	con_putstr("G");
}

static inline void display_cmdline(char* cmdline)
{
	cursor_set_x(0);
	con_putstr("\x1B[J");	// clear line
	con_putstr(cmdline);
}

// process terminal keystroke
// returns echo_on value when enter pressed, 0 otherwise
// must be called at priority lower than console rx/tx h/w interrupt
_Bool process_input(char c)
{
	static enum escseqchar_ {ESC_NONE, ESC_ESC, ESC_CSI}	escseq = ESC_NONE;
	static uint8_t newcmdidx = 0;	// current edited command index
	static uint8_t hisidx = 0;		// index of a selected command from the command history
	static uint8_t oldestcmd = 0;
	static uint8_t clen = 0, ins_idx = 0;
	static _Bool hist_copied;
	static char cmdline[NCMDS][MAXCMDLINE + 1];
	static uint8_t csiarg;
	static char prev_c;
	
	_Bool disprq = 0;
	_Bool retval = 0;
	
	switch (escseq)
	{
	case ESC_ESC:			
		if(c == '[')
		{
			csiarg = 0;
			escseq = ESC_CSI;
		}
		else
			escseq = ESC_NONE;
		break;
	case ESC_CSI:	// ANSI control codes interpretation
		if (c >= '0' && c <= '9')
			csiarg = csiarg * 10 + c - '0';
		else
		{
			switch (c)
			{
			case 'A': // arrow up
				if (hisidx != oldestcmd)
				{
					hisidx = (hisidx + NCMDS - 1) % NCMDS;
					disprq = 1;
				}
				break;
			case 'B': // down
				if (hisidx != newcmdidx)
				{
					hisidx = (hisidx + 1) % NCMDS; 
					disprq = 1;
				}
				break;
			case 'C':	// right
				if (ins_idx < clen)
				{
					ins_idx++;
					con_putstr("\x1B" "9");	// fwd 1 column
				}
				break;
			case 'D': // left
				if (ins_idx > 0)
				{
					ins_idx--;
					con_putstr("\x1B" "6");	// back 1 column
				}
				break;
			case '~':
				if (csiarg == 1)
				{
					// Home
					ins_idx = 0;
					cursor_set_x(ins_idx);
				}
				else if (csiarg == 4)
				{
					// End
					ins_idx = clen;
					cursor_set_x(ins_idx);
				}
			default:
				break;
			}
			if (disprq)
			{
				// display line from history buffer, set clen
				hist_copied = 0;
				display_cmdline(cmdline[hisidx]);
				clen = strlen(cmdline[hisidx]);
				ins_idx = clen;
			}
			escseq = ESC_NONE;
		}
		break;
	case ESC_NONE:
		if (c == 0x1B)
			escseq = ESC_ESC;
		else if ((c == '\b' && ins_idx) || (c >= ' ' && clen < MAXCMDLINE))
		{
			// editing
			if (newcmdidx != hisidx && !hist_copied)
			{
				strcpy(cmdline[newcmdidx], cmdline[hisidx]);
				hist_copied = 1;
			}
			
			if (c == '\b')	// backspace
			{
				if (clen && ins_idx)
				{
					if (--ins_idx == --clen)
					{
						// end of line - just backspace
						cmdline[newcmdidx][clen] = 0;
						con_putstr("\b \b");
					}
					else
					{
						// ins_idx now at character being erased
						cmdline[newcmdidx][ins_idx] = '\b';	// cursor_set_x will be moved back
						con_putstr(&cmdline[newcmdidx][ins_idx]);
						con_putstr(" ");
						cursor_set_x(ins_idx);
						memmove(&cmdline[newcmdidx][ins_idx], &cmdline[newcmdidx][ins_idx + 1], clen - ins_idx + 1);
					}
				}
			}
			else if (c == 127)	// delete
			{
				if (ins_idx < clen)	// if cursor not at eoln
				{
					--clen;
					memmove(&cmdline[newcmdidx][ins_idx], &cmdline[newcmdidx][ins_idx + 1], clen - ins_idx + 1);
					con_putstr(&cmdline[newcmdidx][ins_idx]);
					con_putstr(" ");
					cursor_set_x(ins_idx);
				}
			}
			else	// visible char, place available in buffer
			{
				// insert new char
				if (clen == ins_idx)	// at the end
				{
					// at the end
					cmdline[newcmdidx][clen++] = c;
					cmdline[newcmdidx][clen] = 0;
					ins_idx = clen;
					if (echo_on)
						con_putstr(&cmdline[newcmdidx][clen - 1]);
				}
				else	// in the middle
				{
					memmove(&cmdline[newcmdidx][ins_idx + 1], &cmdline[newcmdidx][ins_idx], clen - ins_idx + 1);
					cmdline[newcmdidx][ins_idx] = c;
					con_putstr(&cmdline[newcmdidx][ins_idx]);
					++ins_idx;
					++clen;
					cursor_set_x(ins_idx);
				}
			}
		}	// editing
		else if (c == '\r' || (prev_c != '\r' && c == '\n'))
		{
			if (echo_on)
				con_put_nl();
			
			uint8_t execidx = newcmdidx;
			
			if (newcmdidx != hisidx && (!hist_copied || strcmp(cmdline[hisidx], cmdline[newcmdidx]) == 0))
			{
				execidx = hisidx;	// execute old command, don't append to history
			}
			else if (cmdline[newcmdidx][0])
			{
				// new non-empty command - add to history
				newcmdidx = (newcmdidx + 1) % NCMDS;	// set next command index
				if (newcmdidx == oldestcmd)
					oldestcmd = (oldestcmd + 1) % NCMDS;
			}
			hisidx = newcmdidx;	// set hist pointer to new command buffer for the next command
			cmdline[newcmdidx][0] = 0; // clear line
			clen = ins_idx = 0;
			
			//if (cmdline[execidx][0])
				cmd_interpret(cmdline[execidx]);
			
			if (echo_on)
				retval = 1;
		}
		break;
	} //switch
	prev_c = c;
	return retval;
}
