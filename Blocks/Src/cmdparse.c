/*
	console command parser with optional hex file record handling
	gbm 2018..2023
*/

#include "cmdproc.h"

//==============================================================================
static inline _Bool is_delim(char c)
{
	return (c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == 0);
}

static inline _Bool is_hex_digit(uint8_t val)
{
	return (val >= '0' && val <= '9') || (val >= 'A' && val <= 'F') 
		|| (val >= 'a' && val <= 'f');
}

static inline uint8_t ihex_digit(uint32_t val)
{
	return val - (val <= '9' ? '0' : (val < 'a' ? 'A' - 10 : 'a' - 10));
}

#define MAXCMDLEN	15
// called when input line completed
uint8_t cmd_interpret(const char* restrict s)
{
	enum cmdi_ret_ {CMDI_OK, CMDI_HEX, CMDI_EMPTY} ret = CMDI_OK;
	enum cmdis_ {CMDI_WAIT, CMDI_LEAD0, CMDI_DNUM, CMDI_HNUM, CMDI_ONUM, CMDI_BNUM, 
		CMDI_CMD, CMDI_HEXLOAD, CMDI_ERR} cmdis = CMDI_WAIT;
	
	uint32_t num = 0;
 	char cmdname[MAXCMDLEN + 1];
	uint8_t cni = 0;
		
#ifdef CMDI_REPEATONEMPTY
	if (*s == 0)
	{
		cmd_repeat();
//		if (lastcmd)
//		{
//			lastcmd();
//			if (run_mode <= RM_ISTEP)
//				while (cpustate == CS_RUN);	
//		}
//		else
//			new_prompt = 1;
		ret = CMDI_EMPTY;
	}
	else
		lastcmd = 0;
#endif
#ifdef CMDI_INTELHEX
	static _Bool hex_error;
	// check for hex record
	if (*s == ':')
	{
		ret = CMDI_HEX;
		// possible Intel Hex
		switch (load_hex_file(s)) // 
		{
		case HL_OK:	// record ok
			echo_on = 0;
			return 0;
		case HL_ENDREC:
			putstr(hex_error ? "Error in Hex file\r\n" : "Hex file loaded\r\n");
			hex_error = 0;
			new_prompt = 1;
			echo_on = 1;
			return 0;
		default:	// error
			hex_error = 1;
			ret = CMDI_OK;
			;
		}
		cmdis = CMDI_HEXLOAD;
	}
#endif
#ifdef CMDI_SRECORD
	else if (s[0] == 'S' && s[1] >= '0' && s[1] <= '9')
	{
		// possible S-Record file
		switch (load_srec_file(s)) // 
		{
		case HL_OK:	// record ok
			echo_on = 0;
			return 0;
		case HL_ENDREC:
			putstr(hex_error ? "Error in S-Record file\r\n" : "S-Record file loaded\r\n");
			hex_error = 0;
			new_prompt = 1;
			echo_on = 1;
			return 0;
		default:	// error
			hex_error = 1;
			;
		}
		cmdis = CMDI_HEXLOAD;
	}
#endif

	while (cmdis != CMDI_HEXLOAD && (*s || (cmdis >= CMDI_DNUM && cmdis <= CMDI_CMD)))
	{
		switch (cmdis)
		{
			case CMDI_WAIT:
				if (is_delim(*s))
					break;
				if (*s == '\'' && *++s)
				{
					stack_push(*s);
					while (!is_delim(*++s));	// unwind
				}
				else if (*s == '"' && *++s)
				{
					// read string
					uint8_t i;
					for (i = 0; i < ARGSTRLEN && *s && *s != '"'; i++)
						argstring[i] = *s++;
					argstring[i] = 0;
				}
				else if (*s >= '0' && *s <= '9')
				{
					num = *s - '0';
					cmdis = *s == '0' ? CMDI_LEAD0 : CMDI_DNUM;
				}
				else
				{
					// command name
					cni = 0;
					cmdis = CMDI_CMD;
					cmdname[cni++] = *s;
				}
				break;
			case CMDI_CMD:
				if (is_delim(*s))
				{
					cmdname[cni] = 0;
					if (!cmd_lookup(cmdname))
					{
						con_putstr("Unrecognized token ");
						con_putstr(cmdname);
						con_put_nl();
					}	
					cmdis = CMDI_WAIT;
				}
				else if (cni < MAXCMDLEN)
					cmdname[cni++] = *s;
				break;
			case CMDI_LEAD0:
				if (*s == 'x' || *s == 'X')
					cmdis = CMDI_HNUM;
				else if (*s == 'b' || *s == 'B')
					cmdis = CMDI_BNUM;
				else if (*s >= '0' && *s <= '7')
				{
					cmdis = CMDI_ONUM;
					num = *s - '0';
				}
				else if (is_delim(*s))
				{
					// just zero
					stack_push(num);	// faster than pushing 0...!
					cmdis = CMDI_WAIT;
				}	
				else
					cmdis = CMDI_ERR;
				break;
			case CMDI_HNUM:
				if (is_hex_digit(*s))
				{
					num <<= 4;
					num += ihex_digit(*s);
					break;
				}
				// no breaks here, will continue to binary handling, then finish or error
			case CMDI_DNUM:
				if (*s >= '0' && *s <= '9')
				{
					num *= 10;
					num += *s - '0';
					break;
				}	
			case CMDI_ONUM:
				if (*s >= '0' && *s <= '7')
				{
					num <<= 3;
					num += *s - '0';
					break;
				}
			case CMDI_BNUM:
				if (*s == '0' || *s == '1')
				{
					num <<= 1;
					num += *s - '0';
					break;
				}
				if (*s == '\'')	// for C23-style numbers with digit separators, added 04'23
					break;
				if (is_delim(*s))	// number successfully scanned
				{
					stack_push(num);
					cmdis = CMDI_WAIT;
				}
				else
				{
					con_putstr("Bad number format\r\n");
					cmdis = CMDI_ERR;
				}
				break;

			case CMDI_ERR:
				if (is_delim(*s))
					cmdis = CMDI_WAIT;
			default:
				break;
		}
		if (*s)
			++s;	
	}
	return ret;
}
