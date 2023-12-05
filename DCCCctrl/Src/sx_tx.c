// DCC transmitter with programming
// gbm 04'2020

#include "board.h"
//#include "dccstate.h"
#include "dcc_tx.h"
#include "config.h"
#include <string.h>

enum sxstate_ {SXS_IDLE, SXS_SYNC0, SXS_SYNC8, SXS_DATA0, SXS_DATA8};

struct sx_ {
	enum sxstate_ state;
	uint8_t bitcnt;
	uint8_t group;
	uint8_t sidx;
	uint16_t sync;	// 12-bit sync/addr datagram
};

struct sx_ sx;

#define SX_LOCOS	112

/* order: sync+addr, loc111, loc95, loc79, loc63, ..., loc15
	first half contains addresses with bit3 = 1, 2nd -> bit 3 = 0
 lsb-first transfer (reversed with respect to datagram pictures in SX docs)
	loco data stored as 1 D7 D6 1 D5 D4 1 D3 D2 1 D1 D0
	sync+addr stored as 1 A0 A1 1 A2 A3 1 S 1 0 0 0 
	baddr: 0 A0 A1 0 A2 A3
*/

static const uint8_t baddr[] = {000, 020, 010, 030, 002, 022, 012, 032,
	001, 021, 011, 031, 003, 023, 013, 033};
uint16_t sxdatagram[128];
	
static void write8sync(uint8_t start)
{
	for (uint8_t i = 0; i < 8; i++)
		sxdatagram[(start + i) * 8] = baddr[start + i] << 6 | 04470;
}

static uint8_t sxidx(uint8_t addr)
{
	// addr: 0..112
	addr = ~addr & 0x7f;
	return (addr & 0xf) << 3 | addr >> 4;
}

void sx_set_datagram(uint8_t idx, uint8_t val)
{
	if (idx < 112)
	{
		sxdatagram[idx] = (val & 3) | (val & 0xc) << 1
			| (val & 0x30) << 2 | (val & 0xc0) << 3
			| 04444;
	}
}

void sx_set_loco_data(uint8_t ln)
{
	if (cd.n.dev[ln].dccaddr < 104)
	{
		sx_set_datagram(cd.n.dev[ln].dccaddr, (loco[ln].dspeed & 0x1f) | loco[ln].rev << 5 | (loco[ln].fun.b[0] & 3) << 6);
	}
}

void sx_preparedata(_Bool sector)
{
	for (uint8_t i = 0; i < 64; i++)
		sxdatagram[sector * 64 + i] = 04444;
	write8sync(sector * 8);
	
	for (uint8_t ln = 0; ln < NDEVICES; ln++)
	{
		uint8_t sxaddr = cd.n.dev[ln].dccaddr;
		if (sxaddr < SX_LOCOS && ((~sxaddr >> 3 & 1) == sector) && cd.n.dev[ln].flags.w)
		{
			uint8_t idx = sxidx(sxaddr);
			//sxdatagram[0] = idx;
			//sxdatagram[64] = idx;
			// sent LSB first
			uint8_t speed = loco[ln].dspeed;
			sxdatagram[idx] = (speed & 3) | (speed & 0xc) << 1 | (speed & 0x10) << 2
				| loco[ln].rev << 7	// 0 - fwd, 1 - rev
				| loco[ln].fun.bf.fl << 9 | (loco[ln].fun.bf.f4_1 & 1) << 10
				| 04444;
		}
	}		
}

void prepare_SX_bit(_Bool bit);

/*
	Packet assembler
	invoked via s/w interrupt by SX bit sender
*/
void SX_PktAs_IRQHandler(void)
{
	sx_preparedata(~sx.sidx >> 6 & 1);
}

// Sender ================================================================
/* Called at the end/start of bit tx period ====================================
Prepares the NEXT bit transfer (the current one is being transmitted now
*/
void sx_tx_next_bit(void)
{
	prepare_SX_bit(sxdatagram[sx.sidx] >> sx.bitcnt & 1);
	
	if (++sx.bitcnt == 12)
	{
		sx.bitcnt = 0;
		++sx.sidx;
		sx.sidx &= 0x7f;
		
		if ((sx.sidx & 0x3f) == 0)
			NVIC_SetPendingIRQ(SX_PktAs_IRQn);
	}
}

// Start Selectrix operation
void sx_start(void)
{
	sx_preparedata(0);
	sx_preparedata(1);
}
