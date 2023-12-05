/*
	DCC command station state data
	gbm 02'2019
*/
#include <stdint.h>
#include "config.h"

#define MAX_PKT_PAYLOAD	8	// for speed&fun packets !!! - 2 B addr + 1 B cmd + speed + 4 B fun
#if 0
#define OPM_PRE_LEN	16
#define PGM_PRE_LEN	20
#define	ONE_HALFPERIOD	58
#define ZERO_HALFPERIOD	(ONE_HALFPERIOD * 2)
#endif

#ifndef	MIN_IDLE_PACKETS
#define	MIN_IDLE_PACKETS	1	// no. of idle packets at the beginning of a refresh cycle
#endif
#ifndef MIN_LOCO_PACKET_SPACING
#define MIN_LOCO_PACKET_SPACING	3	// no. of idle packets equiv. to min. required decoder packet spacing
#endif

#define MAX_PPD	13	// maximum packets per device - speed, 10xF, specialop

/*
packet: 2..8 bytes excl. error check, 1 or 2 address bytes, instruction bytes
	Broadcast address is 0
	28 step format is 01<dir><s0><s4><s3><s2><s1>
*/
struct sendpacket_ {
	uint8_t preamble_len;
	uint8_t pktlen;
	uint8_t pktbuf[MAX_PKT_PAYLOAD];	// without checksum
};
	
/*
	Function state storage structure for F0..68
*/
union fun_ {
	uint32_t w[3];
	struct {
		uint32_t fl:1, f4_1:4, f8_5:4, f12_9:4, f20_13:8, f28_21:8, f31_29:3;
		uint32_t f36_32:5, f44_37:8, f52_45:8, f60_53:8, f63_61:3, f68_64:5;
	} bf;
	uint8_t b[9];
};

enum sphase_ {SPH_SPEED, SPH_F1, SPH_F5, SPH_F9, SPH_F13, SPH_F21, SPH_F29, SPH_F37, SPH_F45, SPH_F53, SPH_F61, SPH_11};

struct loco_state_ {
	_Bool estop;
	_Bool rev;
	uint8_t dspeed;
	uint8_t slimit;
	union fun_ fun;
	uint8_t volume;
	uint8_t phase;	// for DCC sender
};

extern struct loco_state_ loco[NDEVICES];

struct turnout_ {
	uint16_t addr;
	uint8_t subaddr;
	_Bool throw;
};

extern struct turnout_ turnout[NTURNOUTS];
extern uint8_t nturnouts;

struct acc_state_ {
};

// DCC operation state, includes PGM and POM services
struct dccstate_ {
	_Bool pgmmode;
	_Bool pgmmode_rq;	// pgm mode entry request
	volatile _Bool pgmmode_active;	// pgm mode entered
	volatile _Bool pgm_rq;	// programming command request, cleared when programming complete
	volatile _Bool ack, ack_released;
	uint8_t cmdpkt[5];
	uint8_t cmdpktlen;
	_Bool estopall;	// emergency stop for all locos
	_Bool rc_cutout;
	uint8_t dump_rq;
};

// DCC direct mode data
struct dccdm_ {
	_Bool on;
	volatile _Bool drdy;
	uint32_t data;
};

extern struct dccdm_ dcc_direct_mode;

extern struct dccstate_ dccs;

enum hbtype_ {HB_IFX, HB_DRV8871, HB_TB6612, HB_L6206, HB_NTYPES};

extern const char *const hbname[];

struct anstate_ {
	_Bool active;
	_Bool dir_rev;
	_Bool bemf_on;
	uint16_t freq;
	uint16_t steps;
	uint16_t speed;
	uint16_t duty;
	uint16_t bemf_mV;
};

extern struct anstate_ ans;

// Routines defined in dcc_tx_hw.c
// H-bridge hardware control =============================================
void br_set_mode(enum ctrlmode_ mode);
void br_set_andir(_Bool fwd);
void br_set_anduty(uint16_t duty);

void br_cutout_start(void);
void br_cutout_end(void);

void prepare_DCC_bit(_Bool bit);
//========================================================================

void pktdump(const struct sendpacket_ *p);	// defined in vcom1_io.c

// bit preparation routines in protocol-specific modules, defined in dcc_tx.c, called from TX timer interrupt
void dcc_tx_next_bit(void);
void sx_tx_next_bit(void);
void sx_start(void);
