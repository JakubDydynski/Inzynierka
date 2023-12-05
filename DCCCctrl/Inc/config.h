#ifndef CONFIG_H_
#define CONFIG_H_

#include "dccctrl_config.h"

enum ctrlmode_ {CM_OFF, CM_ANCOAST, CM_ANBRAKE, CM_DCC, CM_SX, CM_NMODES};

enum ctrlif_ {CI_MECHANIK, CI_DMX512, CI_DEMO1, CI_NMODES};

extern enum ctrlif_ ctrl_if;
//void init_ctrl_mode(void);

#ifdef DCFG2
// new loco flags structure
// speed packet sending style
enum spdpkt_ {SP_14, SP_28, SP_28F,	// speed 28 and F0..8 in a single packet
	SP_128,	// speed 128
	SP_128F,	//	new RCN packet - speed128 and F0..31
	SP_F256	// 6: Akwi - F1_4 packet with 256 speed in the next octet
};

// loco flags struct should be 32 bits
union locoflags_ {
	uint32_t w;
	struct {
		_Bool
			spd:1,	// send speed packet
			// 10 simple function flags - set if functions implemented
			f1_4:1, f5_8:1, f9_12:1,
			f13_20:1,
			f21_28:1,	// if new speed+fun command is in use, then F29..31 are also supported
			f29_36:1, f37_44:1, f45_52:1, f53_60:1, f61_68:1,
			// additional packet flags
			flag11:1,		//
			volume:1,
			cc:1,	// send consist control command
			// 
			// cmd combine flags - bits 14..15
			flag14:1;	//
			combine_F0_12:1;	// F0..12 in a single packet
			
		uint8_t spdpkt:3;	// enum spdpkt_
		uint8_t maxpktlen:5;
		
		uint8_t unused:2;	// 
		uint8_t cabstyle:2;
		uint8_t f1_4default:4;	// for Akwi light control
	};
};

#define LOCOFLAGSMSK	0x73fff

// 28 step, F1..4: 0x2a1

#define DEVNAMELEN	10	// at least 10 characters

// locomotive config - 16 bytes
struct lococfg_ {
	union locoflags_	flags;	// 4 bytes
	uint16_t dccaddr;		// 2 bytes
	char name[DEVNAMELEN];	// 10 bytes
};

union accflags_ {
	uint32_t w;
	struct {
		_Bool outputaddr:1;
		// todo: device type, "cab" style
	} bit;
};

// same size as loco config
struct acccfg_ {
	union accflags_ flags;
	uint16_t dccaddr;
	char name[DEVNAMELEN];	// 10 bytes
};
	
#endif	// new loco flags

#ifdef DCON1a
#define DEVF_S28	1
#define DEVF_S128	2
#define DEVF_F14	0x10
#define DEVF_F58	0x20
#define DEVF_F912	0x40
#define DEVF_F512	0x80
#define DEVF_F1320	0x100
#define DEVF_F2128	0x200

union locoflags_ {
	uint16_t w;
	struct {
		_Bool speed28:1, speed128:1, fl256:1,
			f1_4s:1,	// f1_4 with speed in a single packet
			// bit 4
			f1_4:1, f5_8:1, f9_12:1,
			f5_12:1,	// f5_12 in a single packet 10
					// if f5_12 and f5_8 are set, f1_12 are transferred in a single packet
			// bit 8
			f13_20:1, f21_28:1,	// 9
			speedlimit:1, flag11:1;	// 12 bits used so far
		uint8_t def_f1_4:4;
	} bit;
};

#define	PH_S	0xf
#define	PH_F1_4	0x10
//#define PH_F5_8	0x20
#define PH_F9_12	0x40
#define PH_F5_12	0xa0	// f5_8 | f5_12
#define	PH_F13_20	0x100
#define PH_F21_28	0x200
#define PH_SLIMIT	0x400

struct devcfg_ {
	uint16_t dccaddr;
	union locoflags_	flags;
};

#ifdef SCFG2	// station config version 2
// station config structure
union stationcfg_ {
	uint8_t b[64];
	struct cdn_ {
		uint8_t ver;
		uint8_t rs485_addr;
		uint8_t ctrl_mode;
		uint8_t nlocos;
		uint8_t nacc;
		uint8_t hbtype:4, hbtype2:4;	// h-bridge type: 0 - BTN/IFX, 1 - TBN6652, 2 - DRV8871
		struct devcfg_ dev[NDEVICES];
	} n;
};
#else
// pre-2023 config structure
union cfgdata_
{
	struct cdn_ {
		uint8_t ver;
		uint8_t rs485_addr;
		uint8_t cmdif;
		uint8_t ndev;
		struct devcfg_ dev[NDEVICES];
		uint8_t hbtype;	// h-bridge type: 0 - BTN/IFX, 1 - TBN6652, 2 - DRV8871
		enum ctrlmode_ mode;	// 
		uint8_t potctrl;	// 0..NCHANNELS-1 - channel controlled by pot; 0xff - inactive
		uint8_t encctrl;	// as above
		uint16_t isensek;	// mA/mV * 1024
		_Bool bidi;	// enable BiDi/Railcom
	} n;
	uint8_t b[sizeof(struct cdn_)];
};
#endif

#else
// old, before dcon1a
struct devcfg_ {
	uint8_t dccaddr;
};

union cfgdata_
{
	struct cdn_ {
		uint8_t ver;
		uint8_t addr;
		uint8_t ndev;
		struct devcfg_ dev[NDEVICES];
		_Bool	lamp_mode;
	} n;
	uint8_t b[sizeof(struct cdn_)];
};
#endif

extern union cfgdata_ cd;

// in dccctrl_config.c
void rstcfg(void);
void loadcfg(void);
void storecfg(void);
void storeautorun(void);

//static const union cfgdata_ dcd = {
//	'A', 0x30, 1,
//	{3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18}
//};
#endif
