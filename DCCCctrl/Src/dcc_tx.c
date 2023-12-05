// DCC transmitter with programming
// gbm 04'2020

#include "board.h"
//#include "dccstate.h"
#include "dcc_tx.h"
#include "config.h"
#include <string.h>

void prepare_DCC_bit(_Bool bit);

#define OPM_PRE_LEN	16	// 14 "one" bits must be transmitted (required), decoder needs 10 ones
#define PGM_PRE_LEN	20

struct loco_state_ loco[NDEVICES];
struct turnout_ turnout[NTURNOUTS];
uint8_t nturnouts;

struct dccstate_ dccs;

static struct sendpacket_ sendpkt = {
	.preamble_len = OPM_PRE_LEN,
	.pktlen = 2,
	.pktbuf[0] = 0xff,
	.pktbuf[1] = 0,
};

static void prepare_idle_packet(void)
{
	sendpkt.pktbuf[0] = 0xff;
	sendpkt.pktbuf[1] = 0;
	sendpkt.pktlen = 2;
	sendpkt.preamble_len = OPM_PRE_LEN;
}

static void prepare_reset_packet(void)
{
	// check!!!!
	sendpkt.pktbuf[0] = 0;
	sendpkt.pktbuf[1] = 0;
	sendpkt.pktlen = 2;
	sendpkt.preamble_len = PGM_PRE_LEN;
}

// New stuff =============================================================
// all possible packet types
enum dcc_pkttype_ {PT_S14, PT_S28, PT_S128,	// classic speed-only
	PT_FS256,	//Akwi
//	PT_S128F0, PT_S128F8, PT_S128F16, PTS128F24,	// new speed with F
	PT_F1_4, PT_F5_8, PT_F9_12,
	PT_F13_20, PT_F21_28,
	PT_F29_36, PT_F37_44, PT_F45_52, PT_F53_60, PT_F61_68,	// new function
	PT_S28F1, PT_F1_8, PT_F5_12, PT_F1_12,	// combined packets
	PT_BSS, PT_BSL,	// binary state
	PT_VOL,	// sound volume
	// 25 types so far
	PT_POM, PT_XPOM,	// programming on main
	PT_PGM,	// service mode programming
};
//========================================================================
// to be overwritten if monitoring interface is present
__attribute__((weak)) void pktdump(const struct sendpacket_ *p)
{
}
// last packet target device for packet spacing detemination =============
#define LDCOUNT 4u
static uint8_t lastdev[LDCOUNT];
static uint8_t ldidx;

static void storeld(uint8_t n)
{
	lastdev[ldidx++] = n;
	ldidx %= LDCOUNT;
}
// utility routines for packet construction ==============================
// add 1 byte to a packet, increment loco phase
static void add2pkt(uint8_t v)
{
	sendpkt.pktbuf[sendpkt.pktlen++] = v;
}

static uint8_t locoidx;

// add 1 byte to a packet, increment loco phase
static void add2pkt1(uint8_t v)
{
	add2pkt(v);
	loco[locoidx].phase++;
}

// add 2 bytes to a packet, increment loco phase
static void add2pkt2(uint8_t c, uint8_t v)
{
	add2pkt(c);
	add2pkt1(v);
}

static uint8_t mkspeed128(uint8_t locoidx)
{
	uint8_t speed = loco[locoidx].dspeed;
	// speed /= 2;	// Katowice !
	if (speed > 126)
		speed = 126;
	if (speed)
		speed++;	// skip step 1 (estop)
	if (loco[locoidx].estop)
		speed = 1;
	return !loco[locoidx].rev << 7 | speed;
}

static uint8_t mkspeed28pkt(uint8_t locoidx)
{
	uint8_t speed = loco[locoidx].dspeed;
	if (speed > 28)
		speed = 28;
	if (speed)
		speed += 3;	// skip step 1 (estop)
	// 0, 4..31
	if (loco[locoidx].estop)
		speed = 2;
	return 2u << 5 | !loco[locoidx].rev << 5 | (speed & 1) << 4 | speed >> 1;
}

static uint8_t mkf14pkt(uint8_t locoidx)
{
	return 4u << 5 | loco[locoidx].fun.bf.f4_1 | loco[locoidx].fun.bf.fl << 4;
}
/*
	Packet assembler
	invoked via s/w interrupt by DCC bit sender at preamble start
	A classic packet may have up to 5 payload bytes + checksum.
	With 6 or more payload bytes, CRC is added before the checksum,
	so the packet is at least 8 bytes long (there are no 7-byte packets in pure DCC standard).
*/
void DCC_PktAs_IRQHandler(void)
{
	static enum {AS_IDLE, AS_LOCO, AS_POM, AS_PGM} astate = AS_IDLE;
	static uint8_t nlocos;
	static uint8_t pkt_repeat;
	
	static enum pgmst_ {PGM_START, PGM_RESET, PGM_CMD, PGM_W4ACK} pgm_seq;
	static uint8_t firstloco;
	static uint16_t oredflags;
#ifndef DCFG2
	static uint16_t seqflags[MAX_PPD + 1];
	static uint8_t seqidx;
#endif	
	static _Bool dumping;
	static _Bool need_another_pass;
	
	switch (astate)
	{
	case AS_IDLE:	// nothing to do, transmitting idle packets
		if (!dccs.dump_rq)
			dumping = 0;
		//pktlen = 2;	// keep sending idle
		if (pkt_repeat)	// must repeat idle
		{
			--pkt_repeat;
		}
		else if (dccs.pgmmode)	// programming mode switch request
		{
			astate = AS_PGM;
			pgm_seq = PGM_START;
		}
		else if (dccs.pgm_rq)
		{
			// POM request, send a packet 3 times
			memcpy(sendpkt.pktbuf, dccs.cmdpkt, dccs.cmdpktlen);
			sendpkt.pktlen = dccs.cmdpktlen;
			pkt_repeat = 3;	// 2 are required
			astate = AS_POM;
		}
		else	// check for normal DCC operation
		{
			if (dccs.dump_rq)
			{
				if (dccs.dump_rq == 1)
					dccs.dump_rq = 0;
				dumping = 1;
			}
			// find first active device
			for (firstloco = 0; firstloco < NDEVICES && !cd.n.dev[firstloco].flags.w; firstloco++);
			if (firstloco < NDEVICES)
			{
				// start locomotive data transfer
				// found at least one active DCC device - decide on seq phases
				oredflags = cd.n.dev[firstloco].flags.w;
				loco[firstloco].phase = 0;	// new - reset loco phase
				for (uint8_t li = firstloco + 1; li < NDEVICES; li++)
				{
					oredflags |= cd.n.dev[li].flags.w;
					loco[li].phase = 0;	// new - reset loco phase
				}
#ifndef DCFG2
				static const uint16_t phaseflags[] = {
					PH_S, PH_F1_4, PH_F5_12, PH_F9_12, PH_F13_20, PH_F21_28, PH_SLIMIT
				};
				memset(seqflags, 0, sizeof(seqflags));
				seqidx = 0;
				for (uint8_t phidx = 0; phidx < sizeof(phaseflags) / sizeof(phaseflags[0]); phidx++)
					if (oredflags & phaseflags[phidx])
						seqflags[seqidx++] = oredflags & phaseflags[phidx];
				seqidx = 0;
#endif
				locoidx = firstloco;
				nlocos = 0;	// no. of locos in the current pass
				need_another_pass = 0;
				astate = AS_LOCO;
				// no packet is prepared here, so idle is transmitted again!!!
			}
			//else	// no active device found - stay in idle
		}
		break;
	case AS_LOCO:	// normal operation mode
		if (pkt_repeat)
		{
			// idle packets between passes through device list
			if (--pkt_repeat == 0)
			{
				locoidx = firstloco;	// restart sequence
				//nlocos = 0;	// no. of locos in the current pass
			}
		}
		if (!pkt_repeat)
		{
			// find next active device to send the packet to
#ifdef DCFG2
			for (; locoidx < NDEVICES && (cd.n.dev[locoidx].flags.w & LOCOFLAGSMSK) < (1u << loco[locoidx].phase);	++locoidx);
#else
			for (; locoidx < NDEVICES && !(cd.n.dev[locoidx].flags.w & seqflags[seqidx]); ++locoidx);
#endif
			if (locoidx == NDEVICES)
			{
				// end of pass through all devices, check if extra idle packet needed
				// due to small number of devices
				pkt_repeat = nlocos + MIN_IDLE_PACKETS < MIN_LOCO_PACKET_SPACING ?
					MIN_LOCO_PACKET_SPACING - MIN_IDLE_PACKETS - nlocos : MIN_IDLE_PACKETS;
				prepare_idle_packet();
				// advance to next phase if needed
#ifdef DCFG2
				if (need_another_pass)
#else
				if (seqflags[++seqidx])
#endif
				{
					locoidx = firstloco;	// start next sequence
					nlocos = 0;	// no. of locos in the current pass
					need_another_pass = 0;
				}
				else
					astate = AS_IDLE;
			}
			else
			{
				// prepare packet
				
				// first, record locomotive number for eventual packet spacing
				storeld(ldidx);
				// start packet assembly - address
				sendpkt.pktlen = 0;
				if (cd.n.dev[locoidx].dccaddr > 127)
					add2pkt(0xc0 | cd.n.dev[locoidx].dccaddr >> 8);
				add2pkt(cd.n.dev[locoidx].dccaddr);
				
#ifdef DCFG2
				// deteremine sequence phase for the current loco
				while (!(cd.n.dev[locoidx].flags.w & LOCOFLAGSMSK & 1u << loco[locoidx].phase))
					phase++;
				//if (nlocos)
				switch (loco[locoidx].phase)
				{
				case SPH_SPEED:
					switch ()
					{
					case SP_14:
					case SP_28:
						add2pkt(mkspeed28pkt(locoidx));
						break;
					case SP_28F:
						add2pkt(mkspeed28pkt(locoidx));
						add2pkt1(4u << 5 | loco[locoidx].fun.bf.f4_1 | loco[locoidx].fun.bf.fl << 4);
						break;
					case SP_128:
						add2pkt2(0x3f, mkspeed128(locoidx));
						break;
					case SP_128F:	// new RCN212 packet
						add2pkt1(4u << 5 | loco[locoidx].fun.bf.f4_1 | loco[locoidx].fun.bf.fl << 4);
						break;
					case SP_F256:	// Akwi F
					}
					break;
				case SPH_F1:
					add2pkt1(4u << 5 | loco[locoidx].fun.bf.f4_1 | loco[locoidx].fun.bf.fl << 4);
					if (!cd.n.dev[locoidx].flags.combine_F0_12)
						break;
				case SPH_F5:
					add2pkt1(5u << 5 | 1u << 4 | loco[locoidx].fun.bf.f8_5);
					if (!cd.n.dev[locoidx].flags.combine_F0_12)
						break;
				case SPH_F9:
					add2pkt1(5u << 5 | 0u << 4 | loco[locoidx].fun.bf.f12_9);
					break;
				case SPH_F13:
					add2pkt2(6u << 5 | 0x1e, loco[locoidx].fun.bf.f20_13);
					break;
				case SPH_F21:
					add2pkt2(6u << 5 | 0x1f, loco[locoidx].fun.bf.f28_21);
					break;
				case SPH_F29:
					add2pkt2(6u << 5 | 0x1f, loco[locoidx].fun.bf.f36_29);
					break;
				case SPH_F37:
					add2pkt2(6u << 5 | 0x1f, loco[locoidx].fun.bf.f44_37);
					break;
				case SPH_F45:
					add2pkt2(6u << 5 | 0x1f, loco[locoidx].fun.bf.f52_45);
					break;
				case SPH_F53:
					add2pkt2(6u << 5 | 0x1f, loco[locoidx].fun.bf.f60_53);
					break;
				case SPH_F61:
					add2pkt2(6u << 5 | 0x1f, loco[locoidx].fun.bf.f68_61);
					break;
				case SPH_CONSIST:
					break;
				}
				if ((cd.n.dev[locoidx].flags.w & LOCOFLAGSMSK) >= (1u << loco[locoidx].phase))
					need_another_pass = 1;
#else
				union locoflags_ lflags;
				lflags.w = cd.n.dev[locoidx].flags.w & seqflags[seqidx];
				
				if (lflags.w & PH_S)
				{
					// speed
					if (cd.n.dev[locoidx].flags.bit.fl256)
					{
						// Akwi: F1..4 and speed
						add2pkt(mkf14pkt(locoidx));
						add2pkt(loco[locoidx].dspeed);
					}
					else if (cd.n.dev[locoidx].flags.bit.speed128)
					{
						add2pkt(0x3f); // 128 step speed
						add2pkt(mkspeed128(locoidx));
					}
					else if (cd.n.dev[locoidx].flags.bit.speed28)
					{
						add2pkt(mkspeed28pkt(locoidx));
					}
					if (cd.n.dev[locoidx].flags.bit.f1_4s)
						add2pkt(mkf14pkt(locoidx));
				}
				else if (lflags.w & PH_F1_4)
				{
					if (lflags.bit.f1_4)
						add2pkt(mkf14pkt(locoidx));
				}
				else if (lflags.w & PH_F5_12)
				{
					if (lflags.bit.f5_8 && lflags.bit.f5_12)	// send F1_12 in a single packet
						add2pkt(mkf14pkt(locoidx));
					if (lflags.bit.f5_8 || lflags.bit.f5_12)
						add2pkt(5u << 5 | 1u << 4 | loco[locoidx].fun.bf.f8_5);
					if (lflags.bit.f5_12)
						add2pkt(5u << 5 | loco[locoidx].fun.bf.f12_9);
				}
				else if (lflags.w & PH_F9_12)
				{
					if (lflags.bit.f9_12)
						add2pkt(5u << 5 | loco[locoidx].fun.bf.f12_9);
				}
				else if (lflags.bit.f13_20)
				{
					add2pkt(6u << 5 | 0x1e);
					add2pkt(loco[locoidx].fun.bf.f20_13);
				}
				else if (lflags.bit.f21_28)
				{
					add2pkt(6u << 5 | 0x1f);
					add2pkt(loco[locoidx].fun.bf.f28_21);
				}
				else if (lflags.bit.speedlimit)
				{
					add2pkt(0x3e);
					add2pkt(loco[locoidx].slimit);
				}
#endif
				++nlocos;
				++locoidx;
			}
			if (dumping)
				pktdump(&sendpkt);	// the only call to pktdump()
		}
		break;
	case AS_POM:
		// programming on main
		if (--pkt_repeat == 0)
		{
			dccs.pgm_rq = 0;
			prepare_idle_packet();
			astate = AS_IDLE;
		}
		else
			sendpkt.pktlen = dccs.cmdpktlen;
		break;
	case AS_PGM:
		// programming mode
		dccs.pgmmode_active = 1;
		if (pkt_repeat)
			--pkt_repeat;
		switch (pgm_seq)
		{
		case PGM_START:
			// 5 reset packets with long preamble
			prepare_reset_packet();
			pkt_repeat = 4;	// NMRA: 3 or more
			pgm_seq = PGM_RESET;
			break;
		case PGM_RESET:	// sending reset packets
			if (!dccs.pgmmode)
			{
				// exit pgm mode
				prepare_idle_packet();
				pkt_repeat = 1;
				astate = AS_IDLE;
				dccs.pgmmode_active = 0;
			}
			else if (!pkt_repeat && dccs.pgm_rq)
			{
				// end of reset seq, send pgm command
				memcpy(sendpkt.pktbuf, dccs.cmdpkt, dccs.cmdpktlen);
				sendpkt.pktlen = dccs.cmdpktlen;
				pkt_repeat = 5;	// 5 or more
				dccs.ack = 0;	// should be cleared by requestor
				dccs.ack_released = 0;
				isenseq_mA = adcval[AC_IS1];
				isense_max = 0;
				pgm_seq = PGM_CMD;
			}
			else
				;//pktlen = 2;	// continue sending reset packet if no request
			break;
		case PGM_CMD:
			// keep sending command until ACK received
			if (pkt_repeat && !dccs.ack)
				sendpkt.pktlen = dccs.cmdpktlen;	// repeat pgm command
			else
			{
				prepare_reset_packet();
				pkt_repeat = 6;	// ACK timeout
				pgm_seq = PGM_W4ACK;
			}
			break;
		case PGM_W4ACK:
			if (!pkt_repeat || (dccs.ack && dccs.ack_released))
			{
				dccs.pgm_rq = 0;
				pgm_seq = PGM_START;
			}
			//pktlen = 2; removed 08.07.22
			break;
		}
		break;
	}
	if (sendpkt.pktlen > 5)
	{
		// add CRC
		add2pkt(0);
	}
	// dump the packet
}

// Sender ================================================================
struct dccdm_ dcc_direct_mode;

enum dccbt_ {DBT_PRE, DBT_DATA, DBT_CHECK, DBT_STOP, DBT_CUTOUT};
/* Called at the end/start of bit tx period ====================================
Prepares the NEXT bit transfer (the current one is being transmitted now
*/
void dcc_tx_next_bit(void)
{
	static uint32_t senddata;
	static uint8_t bitcnt;
	static _Bool sending_data = 0;
	
	if (bitcnt == 0)
	{
		if (dcc_direct_mode.on)
		{
			if (dcc_direct_mode.drdy)
			{
				senddata = dcc_direct_mode.data;
				bitcnt = 32;
				dcc_direct_mode.drdy = 0;
				sending_data = 1;
			}
			else
				prepare_DCC_bit(1); // like preamble
		}
		else
		{
			// preamble or data byte sent, prepare next item
			// init byte/preamble xfer & send first bit
			
			static uint8_t sendidx, chksum;
			static enum dccbt_ sstate;
			
			switch (sstate)
			{
			case DBT_PRE:	// prepare for preamble
				NVIC_SetPendingIRQ(DCC_PktAs_IRQn);
				sending_data = 0;
				bitcnt = sendpkt.preamble_len;
				sstate = DBT_DATA;
				sendidx = 0;
				chksum = 0;
				prepare_DCC_bit(1);
				break;
			case DBT_DATA:
				if (sendpkt.pktlen)
				{
					sending_data = 1;
					bitcnt = 9;
					chksum ^= senddata = sendpkt.pktbuf[sendidx++];
					if (sendidx >= sendpkt.pktlen)
						sstate = DBT_CHECK;
				}
				break;
			case DBT_CHECK:
				bitcnt = 10;
				senddata = chksum << 1 | 1;	// include the stop bit and one more
				sstate = dccs.rc_cutout ? DBT_STOP : DBT_PRE;
				break;
			case DBT_STOP:	// the 1st bit after stop bit is being sent, activate the cutout
				br_cutout_start();
				bitcnt = 0;	// no. of cutout bits
				sstate = DBT_CUTOUT;
			case DBT_CUTOUT:	// deactivate cutout and start preamble
				br_cutout_end();
				sstate = DBT_PRE;
				break;
			}
		}
	}
	
	if (bitcnt)
		--bitcnt;
	if (sending_data)
		prepare_DCC_bit(senddata >> bitcnt & 1);
	// otherwise stay in preamble, sending ones
}

/*
The stuff below should be moved to a separate file, dcc_control.c
*/
#define DEMO_PERIOD	2000

enum ctrlif_ ctrl_if;

void init_ctrl_mode(void)
{
	switch (ctrl_if)
	{
	case CI_DEMO1:
		// set all lamps to fun 5, intensity to dim
		for (uint8_t i = 0; i < NDEVICES; i++)
			if (cd.n.dev[i].flags.w)
			{
				loco[i].dspeed = 5;
				loco[i].fun.w[0] = 5u << 1;
			}
		break;
	default:
		;
	}
}
