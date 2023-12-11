/* 
	DCC controller console commands.
	gbm, 02'2019..04'2020
*/
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "cmdproc.h"
#include "dcc_tx.h"
#include "dcc_auto.h"
#include "config.h"
#include "flash.h"
#ifdef LCONCMD
#include "lconcmd.h"
#endif
#ifdef INA219
#include "ina219.h"
#endif

#define FIRMWARE_VERSION	"n3"

uint8_t (*lastcmd)(void);	// command to repeat (if not null)
extern uint16_t goodframes;

extern uint8_t encval;

static const char * const modename[] = {"Off", "PWM coast", "PWM brake", "DCC", "Selectrix"};
static const char * const cmdifname[] = {"Terminal", "DCC++", "DDW"};
#ifdef CSENSOR
static const char * const csname[] = {"none", "INA219", "INA3221"};
#endif
//==============================================================================
#if 0
#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	con_putchar(ch, 0);
	return ch;
}
#endif

//==============================================================================
// COMMANDS
//==============================================================================
// emergency stop in any mode
uint8_t cmd_estopa(void)
{
	stack_pop();
	return 0;
}

uint8_t cmd_vdisp(void)
{
	char s[100];
	(void) s;
#ifdef INA219
	sprintf(s, "Vin: %u mV, Idcc: %d mA\r\n",
		ina219.voltage_mV, ina219.current_100uA / 10);
	con_putstr(s);
#endif
#ifdef ADCH_IS1
	sprintf(s, "BEMF: %u mV, Is: %u mA, Is_max: %u\r\n",
		adcval[AC_BEMF], adcval[AC_IS1], isense_max);
		//adcavg[AC_BEMF], adcavg[AC_IS1], is1_max, adcavg[AC_IS2], is2_max);
	con_putstr(s);
#endif
#ifdef ADCH_VSUP
	sprintf(s, "Vin: %u mV, Vdd: %u mV, t: %d\r\n"
		" S1: %u mA, S1max: %u, S2: %u mA, S2max: %u\r\n",
		adcval[AC_VSUP], adcval[AC_VREF], adcval[AC_TSEN],
		adcval[AC_IS1], is1_max, adcval[AC_IS2], is2_max);
	con_putstr(s);
#endif
#if 0
	sprintf(s, "Pot: %u, Enc: %u\r\n",
		adcval[AC_POT], encval);
	con_putstr(s);
#endif
	lastcmd = cmd_vdisp;
	return 0;
}

extern uint16_t sxdatagram[128];
//extern uint32_t csid;

uint8_t cmd_statusdisp(void)
{
	char s[100];
	sprintf(s, "H-bridge: %s, isK: %u "
#ifdef CSENSOR
	"sensor: %s, "
#endif
	"mode: %s, command i/f: %s\r\n",
		hbname[cd.n.hbtype], 
		cd.n.isensek,
#ifdef CSENSOR
	csname[ina219.cstype],
#endif
	modename[cd.n.mode],  
		cmdifname[cd.n.cmdif]);
	con_putstr(s);
	cmd_vdisp();
#if 0	// Selectrix data display - for debug
	for (uint8_t j = 0; j < 128; j+=8)
	{
		for (uint8_t i = 0; i < 8; i++)
		{
			sprintf(s, "%04o ", sxdatagram[j + i]);
			con_putstr(s);
		}
		con_put_nl();
	}
#endif
	sprintf(s, "Pot. ctrl channel: %d\r\n", cd.n.potctrl);
	con_putstr(s);
	return 0;
}

// analog mode
uint8_t cmd_andisp(void)
{
	char s[80];
	sprintf(s, "dir: %s, speed: %d %d\r\n", ans.dir_rev ? "rev" : "fwd", ans.speed, ans.duty);
	con_putstr(s);
	return 0;
}

//========================================================================
// pwm mode: brake/coast, bridge type
uint8_t cmd_hbridge(void)
{
	uint8_t hbt = stack_get(0);
	if (hbt < HB_NTYPES)
		cd.n.hbtype = hbt;
	br_set_mode((enum ctrlmode_) cd.n.mode);
	con_putstr(hbname[cd.n.hbtype]);
	con_put_nl();
	return 0;
}

uint8_t cmd_isenseconst(void)
{
	cd.n.isensek = stack_get(0);
	con_putstr(hbname[cd.n.hbtype]);
	con_put_nl();
	return 0;
}

uint8_t cmd_mode(void)
{
	uint8_t cm = stack_get(0);
	if (cm < CM_NMODES)
		cd.n.mode = (enum ctrlmode_)cm;
	if (cd.n.mode == CM_ANCOAST || cd.n.mode == CM_ANBRAKE)
	{
		dccs.pgmmode = 0;
		// suspend DCC operation
		dccs.pgmmode_active = 0;
		
		ans.speed = 0;
		ans.duty = 0;
		ans.dir_rev = 1;
	}
	br_set_mode(cd.n.mode);
	cmd_statusdisp();
	return 0;
}

uint8_t cmd_ctrlif(void)
{
	cd.n.mode = (enum ctrlmode_)stack_get(0);
	return 0;
}

static uint8_t cmd_encctrl(void)
{
	cd.n.encctrl = stack_get(0);
	return 0;
}

static uint8_t cmd_potctrl(void)
{
	cd.n.potctrl = stack_get(0);
	return 0;
}

//========================================================================
// Analog mode commands

static void andisp(void)
{
	char s[80];
	uint32_t freq = SYSCLK_FREQ / ans.steps / (SYSCLK_FREQ / ans.steps / ans.freq);
	sprintf(s, "%" PRIu32 " Hz, %u steps, %s %u, %u\r\n", freq, ans.steps, ans.dir_rev ? "fwd" : "rev", ans.speed, ans.duty);
	con_putstr(s);
}

static uint8_t cmd_aparms(void)
{
	// steps freq
	ans.freq = stack_get(0);
	ans.steps = stack_get(1);
	br_set_mode(cd.n.mode);
	cmd_statusdisp();
	return 0;
}
// DCC programming mode commands =========================================
static _Bool pgmode_check(void)
{
	if (!dccs.pgmmode_active)
		con_putstr("Not in programming mode!\r\n");
	return dccs.pgmmode_active;
}

static _Bool pgm_cmdstart(uint16_t cv, uint8_t b1, uint8_t b3)
{
	if (pgmode_check())
	{
		// store sensed current value
		--cv;
		cv &= 0x3ff;
		dccs.cmdpkt[0] = b1 | cv >> 8;
		dccs.cmdpkt[1] = cv;
		dccs.cmdpkt[2] = b3;
		dccs.cmdpktlen = 3;
		dccs.pgm_rq = 1;
		while (dccs.pgm_rq);
		return dccs.ack;
	}
	else return 0;
}

static _Bool pgm_verify_bit(uint16_t cv, uint8_t bit, _Bool bv)
{
	return pgm_cmdstart(cv, 0x78, 0xe0 | bv << 3 | bit);
}

static _Bool pgm_write_byte(uint16_t cv, uint8_t val)
{
	return pgm_cmdstart(cv, 0x7c, val);
}

static _Bool pgm_verify_byte(uint16_t cv, uint8_t val)
{
	return pgm_cmdstart(cv, 0x74, val);
}

static void cv_write(uint16_t cv, uint8_t val)
{
	con_putstr(pgm_write_byte(cv, val) ? "ok\r\n" : "no ACK\r\n");
}

static const char * const cvname[] = {
	"PriAddr",
	"Vstart",
	"AccRate",
	"DecRate",
	"Vhigh",
	"Vmid",
	"Version",
	"MfgID",
	"PWMperiod",
	"EMFcutout",
	"Timeout",
	"PSconv",
	[16] = "ExtAddrH",
	[17] = "ExtAddrL",
	[18] = "ConsistAddr",
	[28] = "Config"
};

#define NAMEDCVS (sizeof(cvname) / sizeof(cvname[0]))
	
static uint8_t cv_read(uint16_t cvnum)
{
	char s[64];
	sprintf(s, "CV%-3d bin: ", cvnum);
	con_putstr(s);
	uint8_t val  = 0;
	uint8_t i;
	for (i = 0; i < 8; i ++)
	{
		val <<= 1;
		if (pgm_verify_bit(cvnum, 7 - i, 1))
			val++;
		else if (!pgm_verify_bit(cvnum, 7 - i, 0))
			break;
		con_putchar((val & 1) + '0');
	}
	if (i == 8)
	{
		sprintf(s, ", hex: %02x, dec: %3d ", val, val);
		con_putstr(s);
		if (cvnum <= NAMEDCVS && cvname[cvnum - 1][0])
		{
			sprintf(s, "%s, ",cvname[cvnum - 1]);
			con_putstr(s);
		}
	}
	else
		con_putstr(" no ACK\r\n");
	return val;
}

static void cv_verify(uint16_t cvnum, uint8_t val)
{
	con_putstr("verify: ");
	con_putstr(pgm_verify_byte(cvnum, val) ? "ok\r\n" : "no ACK\r\n");
}

static uint8_t cv_readv(uint16_t cvnum)
{
	if (pgmode_check())
	{
		uint8_t val = cv_read(cvnum);
		cv_verify(cvnum, val);
		return val;
	}
	return 0;
}

static uint8_t cmd_cvrd(void)
{
	// cvnum
	uint16_t cvnum = stack_get(0);
	cv_readv(cvnum);
	return 0;
}

static uint8_t cmd_cvrdm(void)
{
	if (pgmode_check())
	{
		// cnt cvnum
		uint16_t cvnum = stack_get(0);
		uint16_t cnt = stack_get(1) & 0x1f;
		while (cnt)
		{
			cv_readv(cvnum++);
			cnt--;
		}
	}
	return 0;
}

static uint8_t cmd_cvrda(void)
{
	cv_readv(1);
	return 0;
}

static uint8_t cmd_cvwr(void)
{
	// val cvnum
	uint16_t cvnum = stack_get(0);
	uint8_t val = stack_get(1);
	cv_write(cvnum, val);
	return 0;
}

static uint8_t cmd_cvwra(void)
{
	// DCC address
	uint16_t val = stack_get(0);
	cv_write(1, val);
	return 0;
}

static uint8_t cmd_cvwrxa(void)
{
	// DCC address
	uint16_t val = stack_get(0);
	cv_write(17, val >> 8 | 0xc0);
	cv_write(18, val);
	return 0;
}

static uint8_t cmd_cvbwr(void)
{
	// val cvnum
	uint16_t cv = stack_get(0);
	uint8_t bit = stack_get(1) & 7;
	_Bool bv = stack_get(2);
	pgm_cmdstart(cv, 0x78, 0xf0 | bv << 3 | bit);
	return 0;
}

static uint8_t cmd_cvvf(void)
{
	// val cvnum
	uint16_t cvnum = stack_get(0);
	uint8_t val = stack_get(1);
	cv_verify(cvnum, val);
	return 0;
}

static uint8_t cmd_cvwrv(void)
{
	// val cvnum
	cmd_cvwr();
	cmd_cvvf();
	return 0;
}

// POM commands ==========================================================
static void pom_cv_write(uint16_t addr, uint16_t cv, uint8_t val)
{
	uint8_t i = 0;
	if (addr > 127)
		dccs.cmdpkt[i++] = 0xc0 | addr >> 8;
	dccs.cmdpkt[i++] = addr;
	--cv;
	cv &= 0x3ff;
	dccs.cmdpkt[i++] = 0xec | cv >> 8;
	dccs.cmdpkt[i++] = cv;
	dccs.cmdpkt[i++] = val;
	dccs.cmdpktlen = i;
	dccs.pgm_rq = 1;
	while (dccs.pgm_rq);
}

static uint8_t cmd_pomcvwr(void)
{
	if (dccs.pgmmode_active)
		con_putstr("Pgm mode!\r\n");
	else
	{
		// POM
		uint16_t addr = stack_pop();
		uint16_t cv = stack_pop();
		uint8_t val = stack_pop();
		pom_cv_write(addr, cv, val);
		return dccs.ack;
	}
	return 0;
}

// controller parameters =================================================
static uint8_t cmd_addrdisp(void)
{
	char resp[64];
	
	sprintf(resp, "addr: 0x%02x, ctrl mode: %u\r\n", cd.n.rs485_addr, cd.n.cmdif);//, nframes, goodframes);
	con_putstr(resp);
	return 0;
}

static uint8_t cmd_addr(void)
{
	cd.n.rs485_addr = stack_pop();
	return 0;
}
// DCC device control ------------------------------------------------------------------
static uint8_t get_devidx(void)
{
	return stack_pop() % NDEVICES;
}

static void devdisp(uint8_t idx)
{
	char s[80];
	sprintf(s, "%2u: addr %4u, fl %04x, %s %3u, fun %02" PRIx32 " %08" PRIx32 " %08" PRIx32 "\r\n",
		idx, cd.n.dev[idx].dccaddr, cd.n.dev[idx].flags.w,
	loco[idx].rev ? "Rev" : "Fwd", loco[idx].dspeed,
	loco[idx].fun.w[2], loco[idx].fun.w[1], loco[idx].fun.w[0]);
	con_putstr(s);
}

static uint8_t cmd_devdisp(void)
{
	// idx
	uint8_t idx = get_devidx();
	devdisp(idx);
	return 0;
}

static uint8_t cmd_devdef(void)
{
	// DCCaddr idx
	uint8_t devidx = get_devidx();
	uint16_t addr = stack_pop();
	if (addr < 0xc000 && addr > 127)
		addr = 0xc000 + (addr & 0x3fff);	// implicit long address
	uint32_t devflags = stack_pop();
	cd.n.dev[devidx].dccaddr = addr;
	cd.n.dev[devidx].flags.w = devflags;
	loco[devidx].dspeed = 0;
	loco[devidx].fun.w[0] = cd.n.dev[devidx].flags.bit.def_f1_4 << 1;
	return 0;
}

#ifdef LCONCMD
static void lampdef(uint8_t devidx, uint8_t addr)
{
	// DCCaddr idx
	cd.n.dev[devidx].dccaddr = addr;
	cd.n.dev[devidx].flags.w = 0;
	cd.n.dev[devidx].flags.bit.def_f1_4 = 5;
#ifdef DCON1a
//	cd.n.dev[devidx].flags.bit.f1_4s = 1; no need
	cd.n.dev[devidx].flags.bit.fl256 = 1;
#else
	cd.n.dev[devidx].flags.bit.f1_4 = 1;
	cd.n.dev[devidx].flags.bit.speed128 = 1;
#endif
	loco[devidx].dspeed = 0;
	loco[devidx].fun.w[0] = cd.n.dev[devidx].flags.bit.def_f1_4 << 1;
	//loco[devidx].dspeed = sizeof(cd.n.dev[devidx]);
}

static uint8_t cmd_lampdef(void)
{
	// DCCaddr idx
	uint8_t devidx = get_devidx();
	uint16_t addr = stack_pop() & 0x3fff;
	lampdef(devidx, addr);
	return 0;
}
static uint8_t cmd_lampdefgroup(void)
{
	// DCCaddr idx
	uint8_t nlamps = stack_pop();
	for (uint8_t i = 1; i <= nlamps; i++)
		lampdef(i < NDEVICES ? i : 0, i);
	return 0;
}
#endif
// set speed for group
static uint8_t cmd_speedgroup(void)
{
	// DCCaddr idx
	uint8_t last = stack_pop() % NDEVICES;
	uint8_t i = stack_pop() % NDEVICES;
	uint8_t speed = stack_pop();
	for (; i <= last; i++)
		loco[i].dspeed = speed;
	return 0;
}

// delete group of devices
static uint8_t cmd_devdelgroup(void)
{
	// DCCaddr idx
	uint8_t last = stack_pop() % NDEVICES;
	for (uint8_t i = stack_pop() % NDEVICES; i <= last; i++)
	{
		memset(&cd.n.dev[i], 0, sizeof(cd.n.dev[i]));
		loco[i].dspeed = 0;
		memset(&loco[i].fun, 0, sizeof(loco[i].fun));
	}
	return 0;
}
// list all devices - no args
static uint8_t cmd_devlist(void)
{
	for (uint8_t idx = 0; idx < NDEVICES; idx++)
		if (cd.n.dev[idx].dccaddr || cd.n.dev[idx].flags.w)
			devdisp(idx);
	return 0;
}

static uint8_t cmd_devoff(void)
{
	// idx
	uint8_t devidx = get_devidx();
	cd.n.dev[devidx].flags.w = 0;
	return 0;
}

#if 0
static uint8_t cmd_devon(void)
{
	// idx
	uint8_t devidx = get_devidx();
	cd.n.dev[devidx].flags.w = 1;
	return 0;
}
#endif
// Function control ======================================================
static uint8_t cmd_funbitfield(void)
{
	// value width start devidx
	uint8_t devidx = get_devidx();
	uint8_t start = stack_pop();
	uint8_t width = stack_pop() % 32;
	uint8_t val = stack_pop();
	uint8_t widx = start / 32;
	start %= 32;
	uint32_t lmask = 0xffffffff >> (32 - width) << start;
	loco[devidx].fun.w[widx] = (loco[devidx].fun.w[widx] & ~lmask) | (val << start & lmask);
	if (start + width > 32)
	{
		val >>= 32 - start;
		width = start + width - 32;
		uint32_t hmask = 0xffffffff >> (32 - width);
		++widx;
		loco[devidx].fun.w[widx] = (loco[devidx].fun.w[widx] & ~hmask) | (val & hmask);
	}
	devdisp(devidx);

	return 0;
}

static uint8_t cmd_fun(void)
{
	// fstate idx
	uint8_t devidx = get_devidx();
	loco[devidx].fun.w[0] = stack_pop();
	devdisp(devidx);

	return 0;
}

static uint8_t cmd_fun32(void)
{
	// fstate idx
	uint8_t devidx = get_devidx();
	loco[devidx].fun.w[1] = stack_pop();
	devdisp(devidx);

	return 0;
}

static uint8_t cmd_fun64(void)
{
	// fstate idx
	uint8_t devidx = get_devidx();
	loco[devidx].fun.w[2] = stack_pop();
	devdisp(devidx);

	return 0;
}

// turn on for a specified time
static uint8_t cmd_fdly(void)
{
	// val fnum idx
	uint8_t devidx = get_devidx();
	uint8_t fnum = stack_pop();
	uint8_t ontime = stack_pop();
	if (ontime == 0)
		ontime = 1;
	else if (ontime > 10)
		ontime = 10;
	if (fnum > 68)
		fnum = 68;
	loco[devidx].fun.w[fnum / 32] |= 1u << (fnum % 32);
	devdisp(devidx);
	// delay - change !!!
	void HAL_Delay(__IO uint32_t Delay);
	HAL_Delay(ontime * 1000);
	loco[devidx].fun.w[fnum / 32] &= ~(1u << (fnum % 32));
	devdisp(devidx);
	return 0;
}

static uint8_t cmd_ftoggle(void)
{
	// val fnum idx
	uint8_t devidx = get_devidx();
	uint8_t fnum = stack_pop();
	if (fnum < 32)
		loco[devidx].fun.w[0] ^= 1u << fnum;
	if (fnum >= 28 && fnum <= 68)
	{
		fnum += 3;
		loco[devidx].fun.w[fnum / 32] ^= 1u << (fnum % 32);
	}
	devdisp(devidx);
	return 0;
}

// set single function by number
static uint8_t cmd_fx(void)
{
	// val fnum idx
	uint8_t devidx = get_devidx();
	uint8_t fnum = stack_pop();
	_Bool on = stack_pop();
	if (fnum < 32)
	{
		if (on)
			loco[devidx].fun.w[0] |= 1u << fnum;
		else
			loco[devidx].fun.w[0] &= ~(1u << fnum);
	}
	if (fnum >= 28 && fnum <= 68)
	{
		fnum += 3;
		if (on)
			loco[devidx].fun.w[fnum / 32] |= 1u << (fnum % 32);
		else
			loco[devidx].fun.w[fnum / 32] &= ~(1u << (fnum % 32));
	}
	devdisp(devidx);
	return 0;
}

// operatiing mode, diagnostics ==========================================
static uint8_t cmd_opmode(void)
{
	dccs.pgmmode = 0;
	return 0;
}

static uint8_t cmd_pgmmode(void)
{
	dccs.pgmmode = 1;
	return 0;
}

static uint8_t cmd_pktdump(void)
{
	// 1 - zrzut jednego przebiegu, > 1 - zrzut ciagly
	dccs.dump_rq = stack_pop();;
	return 0;
}

static uint8_t cmd_start(void)
{
	return 0;
}

// addr mask type bknum
static uint8_t cmd_stop(void)
{
	return 0;
}
static uint8_t cmd_bidi(void)
{
	cd.n.bidi = stack_pop();
	return 0;
}

static uint8_t cmd_bemf(void)
{
	ans.bemf_on = stack_pop();
	return 0;
}

// Speed control =========================================================
static uint8_t cmd_dir(void)
{
	uint8_t devidx = get_devidx();
	loco[devidx].rev = stack_pop();
	devdisp(devidx);
	return 0;
}

static uint8_t cmd_estop(void)
{
	if (ans.active)
		;
	else
	{
		uint8_t devidx = get_devidx();
		loco[devidx].estop = stack_pop();;
		devdisp(devidx);
	}
	return 0;
}

static uint8_t cmd_fwd(void)
{
	if (ans.active)
		br_set_andir(1);
	else
	{
		uint8_t devidx = get_devidx();
		loco[devidx].rev = 0;
		devdisp(devidx);
	}
	return 0;
}

static uint8_t cmd_rev(void)
{
	if (ans.active)
		br_set_andir(0);
	else
	{
		uint8_t devidx = get_devidx();
		loco[devidx].rev = 1;
		devdisp(devidx);
	}
	return 0;
}

static uint8_t cmd_speed(void)
{
	if (ans.active)
	{
		uint16_t speed = stack_pop();
		if (ans.bemf_on)
		{
			if (speed <= 100u)
				ans.speed = speed;
		}
		else
		{
			ans.speed = speed;
			br_set_anduty(ans.duty = ans.speed);
		}
		andisp();
	}
	else
	{
		uint8_t devidx = get_devidx();
		uint8_t speed = stack_pop();
		loco[devidx].dspeed = speed;
		devdisp(devidx);
	}
	return 0;
}

static uint8_t cmd_vol(void)
{
	uint8_t devidx = get_devidx();
	uint8_t vol = stack_pop();
	loco[devidx].volume = vol;
	devdisp(devidx);
	return 0;
}
// Autorun commands ======================================================
static uint8_t cmd_arctrl(void)
{
	if (stack_pop())
		autorun_start();
	else
		autorun_stop();
	return 0;
}

static void print_ar(uint8_t i)
{
	char s[80];
	sprintf(s, "%2d %3d s: %2d %s %3d %08" PRIx32 " %s\r\n",
		i, autopgm[i].stime, autopgm[i].locnum, autopgm[i].rev ? "rev" : "fwd",
		autopgm[i].speed, autopgm[i].f0_28, autopgm[i].itof ? "tof" : "aut");
	con_putstr(s);
}
// define autorun step
static uint8_t cmd_ardef(void)
{
	// fun speed rev loco time ar_slot
	uint8_t pos = stack_pop();
	uint8_t t = stack_pop();
	uint8_t l = stack_pop();
	_Bool rev = stack_pop();
	uint8_t speed = stack_pop();
	uint32_t fun = stack_pop();
	_Bool itof = stack_pop();
	
	if (pos < NAUTOSTEPS && l < NDEVICES)
	{
		autopgm[pos].stime = t;
		autopgm[pos].locnum = l;
		autopgm[pos].rev = rev;
		autopgm[pos].speed = speed;
		autopgm[pos].f0_28 = fun;
		autopgm[pos].itof = itof;
		print_ar(pos);
	}
	return 0;
}

static uint8_t cmd_arsave(void)
{
#ifndef CFG_SINGLE_PAGE
	storeautorun();
#endif
	return 0;
}
// define six-step shuttle operation for autorun
static uint8_t cmd_arshuttle(void)
{
	// fun full_speed loco stop_time run_rime start_slot
	uint8_t pos = stack_pop();
	uint8_t tr = stack_pop();
	uint8_t ts = stack_pop();
	uint8_t l = stack_pop();
	uint8_t speed = stack_pop();
	uint32_t fun = stack_pop();
	
	if (pos < NAUTOSTEPS && l < NDEVICES)
	{
		struct autostep_ step = {.stime = ts, .locnum = l, .rev = 0, .speed = 0, .f0_28 = fun}; 
		autopgm[pos + 0] = step;
		autopgm[pos + 1] = step;
		autopgm[pos + 1].stime = tr;
		autopgm[pos + 1].speed = speed;
		autopgm[pos + 2] = step;
		step.rev = 1;
		autopgm[pos + 3] = step;
		autopgm[pos + 4] = step;
		autopgm[pos + 4].stime = tr;
		autopgm[pos + 4].speed = speed;
		autopgm[pos + 5] = step;
	}
	return 0;
}

extern volatile uint16_t seccnt;

static uint8_t cmd_arlist(void)
{
	for (uint8_t i = 0; i < NAUTOSTEPS && autopgm[i].stime; i++)
		print_ar(i);
	char s[30];
	sprintf(s, "step: %d, time: %d, seccnt: %d\r\n", curr_step, curr_step_time, seccnt);
	con_putstr(s);
	return 0;
}

static uint8_t cmd_save(void)
{
	storecfg();
	return 0;
}

static uint8_t cmd_rstcfg(void)
{
	rstcfg();
	return 0;
}

static uint8_t cmd_boot(void)
{
	hw_boot();
	return 0;
}

#ifdef LCONCMD
static uint8_t cmd_ttest(void)
{
	// send my intensities to a specified controller
	uint8_t raddr = stack_pop();
	
	uint8_t pkt[40];
	pkt[0] = raddr;
	pkt[1] = 35;
	pkt[2] = 0x17;	// multiple channel bri
	for (uint8_t i = 0; i < NDEVICES; i++)
		pkt[3 + i] = loco[i].dspeed;
	send_mb_frame(pkt);

	return 0;
}
#endif
//------------------------------------------------------------------------------
extern uint32_t usbtimes;
extern uint32_t usbchars;

static _Bool case_sens;

static uint8_t cmd_casesens(void)
{
	case_sens = stack_pop();
	return 0;
}

// help and utilities ====================================================
static uint8_t cmd_help(void)
{
	if (dccs.pgmmode)
		con_putstr("Programming mode commands:\r\n"
		"\t"		"\t"		"\t"		"dl\t"		"list devices\r\n"
		"\t"		"\t"		"<idx>\t"	"dd\t"		"display device info\r\n"
		"<flags>\t"	"<addr>\t"	"<idx>\t"	"ddef\t"	"define device\r\n"
#ifdef LCONCMD
		"\t"		"<addr>\t"	"<idx>\t"	"ldef\t"	"define lamp\r\n"
		"\t"		"\t"		"<n>\t"		"ldg\t"		"define n lamps\r\n"
#endif
		"\t"		"\t"		"\t"		"opm\t"		"DCC operation mode\r\n"
		"\t"		"\t"		"<cv>\t"	"rd\t"		"CV read\r\n"
		"\t"		"\t"		"\t"		"rda\t"		"read short address\r\n"
		"\t"		"<cnt>\t"	"<cv>\t"	"rdm\t"		"multiple CV read\r\n"
		"\t"		"<val>\t"	"<cv>\t"	"vf\t"		"CV verify\r\n"
		"\t"		"<val>\t"	"<cv>\t"	"wr\t"		"CV write\r\n"
		"\t"		"\t"		"<val>\t"	"wra\t"		"write short address\r\n"
		"<val>\t"	"<pos>\t"	"<cv>\t"	"bwr\t"		"CV bit write\r\n"
		);
	else
		con_putstr("Commands:\r\n"
		"\t"		"\t"		"\t"		"?par\t"	"show controller parms\r\n"
		"\t"		"\t"		"\t"		"?s\t"		"show controller status\r\n"
		"\t"		"\t"		"\t"		"?v\t"		"show voltage/current\r\n"
		"\t"		"\t"		"\t"		"?an\t"		"show analog mode status\r\n"
		"\t"		"\t"		"<addr>\t"	"addr\t"	"set controller address\r\n"
		"\t"		"\t"		"<mode>\t"	"mode\t"	"set mode 0..4->off, an1, an2, DCC, SX\r\n"
		"\t"		"\t"		"\t"		"dl\t"		"list devices\r\n"
		"\t"		"\t"		"<idx>\t"	"dd\t"		"display device info\r\n"
		"\t"		"<first>\t"	"<last>\t"	"ddg\t"		"delete device group\r\n"
		"<flags>\t"	"<addr>\t"	"<idx>\t"	"ddef\t"	"define device\r\n"
#ifdef LCONCMD
		"\t"		"<addr>\t"	"<idx>\t"	"ldef\t"	"define lamp\r\n"
#endif
#ifndef SAVESPACE
		"\t"		"\t"		"<idx>\t"	"doff\t"	"device control off\r\n"
		"\t"		"\t"		"<idx>\t"	"don\t"		"device control on\r\n"
		"\t"		"<fwd>\t"	"<idx>\t"	"dir\t"		"direction\r\n"
		"\t"		"\t"		"<idx>\t"	"fwd/rev\t"	"direction\r\n"
		"\t"		"<spd>\t"	"<idx>\t"	"spd\t"		"speed\r\n"
		"\t"		"<vol>\t"	"<idx>\t"	"vol\t"		"sound volume\r\n"
		"\t"		"<fun>\t"	"<idx>\t"	"fun/fun32/fun64\t"		"functions F31..0,F63..32, F68..64 set\r\n"
		"<on>\t"	"<Fn>\t"	"<idx>\t"	"fx\t"		"Fx function on/off\r\n"
		"<val>\t"	"<cv>\t"	"<addr>\t"	"pomwr\t"	"CV write in POM mode\r\n"
		"\t"		"\t"		"\t"		"pgm\t"		"DCC programming mode\r\n"
		"\t"		"<steps>\t"	"<freq>\t"	"ap\t"		"analog mode parameters\r\n"
		"\t"		"\t"		"\t"		"save\t"	"save setup data\r\n"
#endif
		"<on> ar, <fun><sp><fwd><loc><time><pos>ard\r\n"
		"arl, arsave\r\n"
		);
	return 0;
}

const struct cmd_ cmdtab[] = {
	{"!", 		cmd_estopa},
	{"?", 		cmd_help},		// display help
	{"?addr",	cmd_addrdisp},		// 
	{"?an",		cmd_andisp},		// 
	{"?par",	cmd_addrdisp},		// 
	{"?s",	 	cmd_statusdisp},
	{"?v",	 	cmd_vdisp},
	{"_", 		cmd_casesens},
	{"addr", 	cmd_addr},
	{"ap",	 	cmd_aparms},
	{"ar",		cmd_arctrl},
	{"ard",		cmd_ardef},
	{"arl",		cmd_arlist},
	{"arsave",	cmd_arsave},
	{"arsh",	cmd_arshuttle},
	{"bemf",	cmd_bemf},
	{"bidi",	cmd_bidi},
	{"boot",	cmd_boot},
	{"bwr",	 	cmd_cvbwr},
	{"ci",		cmd_ctrlif},
	{"dd",		cmd_devdisp},
	{"ddef",	cmd_devdef},
	{"ddg",		cmd_devdelgroup},
	{"dir", 	cmd_dir},
	{"dl",		cmd_devlist},
	{"doff",	cmd_devoff},
//	{"don",		cmd_devon},
	{"enc",		cmd_encctrl},
	{"estop",	cmd_estop},
	{"estopa",	cmd_estopa},
	{"fb",		cmd_funbitfield},
	{"fd",		cmd_fdly},
	{"ft",		cmd_ftoggle},
	{"fun",		cmd_fun},
	{"fun32",	cmd_fun32},
	{"fun64",	cmd_fun64},
	{"fwd",		cmd_fwd},
	{"fx",		cmd_fx},
	{"hb",	 	cmd_hbridge},
	{"isk",	 	cmd_isenseconst},
#ifdef LCONCMD
	{"kbq",		cmd_kbq},
	{"ldef",	cmd_lampdef},
	{"ldg",		cmd_lampdefgroup},
	{"li",		cmd_listen},	// 
#endif
	{"mode", 	cmd_mode},
	{"opm", 	cmd_opmode},
	{"pd", 		cmd_pktdump},
	{"pgm", 	cmd_pgmmode},
	{"pomwr",	cmd_pomcvwr},
	{"pot",		cmd_potctrl},
	{"rd",	 	cmd_cvrd},
	{"rda",	 	cmd_cvrda},
	{"rdm",	 	cmd_cvrdm},
	{"rev",	 	cmd_rev},
	{"rstcfg", 	cmd_rstcfg},
	{"save", 	cmd_save},
	{"spd", 	cmd_speed},
	{"spg", 	cmd_speedgroup},
	{"start", 	cmd_start},
	{"stop", 	cmd_stop},
#ifdef LCONCMD
	{"tltn",	cmd_tltn},
	{"tt", 		cmd_ttest},
	{"txkeep",	cmd_txkeep},
#endif
	{"vf",	 	cmd_cvvf},
	{"vol",	 	cmd_vol},
	{"wr",	 	cmd_cvwr},
	{"wra",	 	cmd_cvwra},
	{"wrv",	 	cmd_cvwrv},
	{"wrxa",	cmd_cvwrxa},
	{0, 0}
};

// device-specific command lookup, calls std command lookup defined in cmdproc
_Bool cmd_lookup(const char *cmds)
{
	_Bool ret = 0;
	if (!ret)
	{
		if (!case_sens)
		{
			//convert command string to lower case
			char lccmd[16];
			uint8_t i;
			for (i = 0; i < 15 && cmds[i]; i++)
				lccmd[i] = cmds[i] >= 'A' && cmds[i] <= 'Z' ? cmds[i] + 'a' - 'A' : cmds[i];
			lccmd[i] = 0;
			ret = cmd_tlookup(lccmd, cmdtab);
		}
		else
			ret = cmd_tlookup(cmds, cmdtab);
	}
	if (!ret)
		ret = stdcmd_lookup(cmds);
	
//	new_prompt = 1;
	return ret;
}

void cmd_repeat(void)
{
	if (lastcmd)
		lastcmd();
//	new_prompt = 1;
}
//==============================================================================
// PROMPT
//==============================================================================
// NOT called from this module
void display_prompt(void)
{
	cmdline_start = 2;
	con_putchar('\r');
	con_putchar('>');
}

void display_signon(void)
{
	con_putstr("\r\n\r\n" DEVNAME
	"  fv." FIRMWARE_VERSION "  (? - command help)\r\n\r\n");
	display_prompt();
	echo_on = 1;
}
