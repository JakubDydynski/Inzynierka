/* 
	DCC controller config data storage.
	gbm, 02'2019..04'2020
*/
#include <stdint.h>
#include <string.h>
#include "board.h"
#include "dcc_tx.h"
#include "config.h"
#include "dcc_auto.h"
#include "flash.h"

union cfgdata_ __attribute__((aligned)) cd;
//static union cfgdata_ oldcd;

const union cfgdata_ __attribute__((aligned)) defcd = {
	.n = {
		.ver = 'B',
		.dev = {
			{0},
			{1},
			{2},
			{3},
			{4},
			{5},
			{6},
			{7},
			{8},
			{9},
			{10},
			{11},
			{12},
			{13},
			{14},
			{15},
			{16},
			{17},
			{18},
			{19},
			{21},
			{22},
			{23},
			{24},
			{25},
			{26},
			{27},
			{28},
			{29},
			{30},
			{31}
		},
		.potctrl = 0xff,
		.encctrl = 0xff
	}
};

#define savedcfg (*(union cfgdata_ *)CFGADDR)
#define savedauto ((struct autostep_ *)ARUNADDR)
	
void rstcfg(void)
{
	flash_erase_page((void *)CFGADDR);
	cd = defcd;
}

void loadcfg(void)
{
	if (savedcfg.n.ver == 'A' || savedcfg.n.ver == 'B')
	{
		cd = savedcfg;
	}
	else
		cd = defcd;
	
	if (savedcfg.n.ver == 'A')
	{
		// update cfg to version B
		cd = defcd;
	}
	for (uint8_t i = 0; i < NDEVICES; i++)
		loco[i].fun.bf.f4_1 = cd.n.dev[i].flags.bit.def_f1_4;	// send F3, F1
	
	if (savedauto[0].stime != 0xff)
	{
		memcpy(autopgm, savedauto, sizeof(autopgm));
	}
}

void storecfg(void)
{
	// erase cfg page
	flash_erase_page((void *)CFGADDR);
	// write
	for (uint32_t a = 0; a < (sizeof(cd) + 7) / 8; a++)
	{
		flash_write64((uint32_t *)(CFGADDR + a * 8), (uint32_t *)(&cd.b[a * 8]));
	}
#ifdef CFG_SINGLE_PAGE
	for (uint32_t a = 0; a < (sizeof(autopgm) + 7) / 8; a++)
	{
		flash_write64((uint32_t *)(ARUNADDR + a * 8), (uint32_t *)((uint8_t *)autopgm + a * 8));
	}
#endif
}

void storeautorun(void)
{
#ifndef CFG_SINGLE_PAGE
	// erase cfg page
	flash_erase_page((void *)ARUNADDR);
	// write
	for (uint32_t a = 0; a < (sizeof(autopgm) + 7) / 8; a++)
	{
		flash_write64((uint32_t *)(ARUNADDR + a * 8), (uint32_t *)((uint8_t *)autopgm + a * 8));
	}
#endif
}
