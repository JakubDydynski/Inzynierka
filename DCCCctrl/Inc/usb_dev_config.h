/*
	Device-specific config file
	gbm 2017
*/

#ifndef __USB_DEV_CONFIG_H
#define __USB_DEV_CONFIG_H

#include <stdint.h>

#define VID	0x483	// ST Microelectronics

//#define MSC_CDC
//#define MSC_2CDC
//#define ONE_CDC
#define TWO_CDC
//#define MSC_DOUBLE_CDC
//#define MSC_CDC_PRINTER
//#define PRINTER_ONLY

#define USBD_CFG_MAXCURRENT	100

#define USBD_CLASS_MSC

#define USE_INT_ENDPOINT	1

#define MSC_DATA_PACKET_SIZE	64
#define CDC_DATA_PACKET_SIZE	64
#define CDC_INT_PACKET_SIZE	8
#define PRN_DATA_PACKET_SIZE	64
#define EP0_MAX_PACKET_SIZE	64

#define	PRN_SUBCLASS_1	1
#define	PRN_PROTOCOL_1	1

//========================================================================
#ifdef MSC_2CDC
#define PID	0x7fff	// my own
#define USE_MSC 1
#define CDC_CHANNELS	2
#endif

#ifdef ONE_CDC
#define PID	0x7fff	// my own
#define CDC_CHANNELS	1
#endif

#ifdef TWO_CDC
#define PID	0x7fff	// my own
#define CDC_CHANNELS	2
#endif

//========================================================================
#ifdef MSC_CDC
#define PID	0x7fff	// my own
#define USE_MSC 1
#define CDC_CHANNELS	1
#endif
//========================================================================
#ifdef MSC_CDC_PRINTER
#define USE_PRN
#define NEPPAIRS	4
#define CFGDESC_ cfgdesc_msc_cdc_prn_
#endif

//========================================================================
enum usbd_ifnum_ {
#ifdef USE_MSC
	IFNUM_MSC,
#endif
#ifdef CDC_CHANNELS
	IFNUM_CDC0_CONTROL, IFNUM_CDC0_DATA,
#if CDC_CHANNELS > 1
	IFNUM_CDC1_CONTROL, IFNUM_CDC1_DATA,
#endif	// CDC_CHANNELS > 1
#endif	// any CDC_CHANNELS
#ifdef USE_PRN
	IFNUM_PRN,
#endif
	USBD_MAX_NUM_INTERFACES
};

enum usbd_epnum_ {
	EP0_OUT_EP,
#ifdef USE_MSC
	MSC_DATA_OUT_EP,
#endif
#ifdef CDC_CHANNELS
	CDC0_DATA_OUT_EP,
#if CDC_CHANNELS > 1
	CDC1_DATA_OUT_EP,
#endif
#endif
	EP0_IN_EP = 0x80,
#ifdef USE_MSC
	MSC_DATA_IN_EP,
#endif
#ifdef CDC_CHANNELS
	CDC0_DATA_IN_EP,
#if CDC_CHANNELS > 1
	CDC1_DATA_IN_EP,
#endif
	CDC0_INT_IN_EP,
#if CDC_CHANNELS > 1
	CDC1_INT_IN_EP,
#endif
#endif
#ifdef USE_PRN
#endif
	NUM_IN_EPS
};

#define NEPPAIRS	(NUM_IN_EPS & 0xf)
	
#define CDC_DATA_EP_DELTA	(CDC1_DATA_IN_EP - CDC0_DATA_IN_EP)

#define CFGDESC_ cfgdesc_msc_2cdc_

//========================================================================
#ifdef PRINTER_ONLY
#define PID	0x7fee	// my own
#define USE_PRN	// to enable class requests
enum usbd_ifnum_ {IFNUM_PRN, IFNUM_NINTERFACES};
#define NEPPAIRS	2
enum usbd_epnum_ {EP0_OUT_EP, PRN_DATA_OUT_EP, MSC_DATA_OUT_EP,
	EP0_IN_EP = 0x80, MSC_DATA_IN_EP};
#define CFGDESC_ cfgdesc_prn_
#endif

// device number within composite device
//#define SUBDEV_NUM_CDC0		0
//#define SUBDEV_NUM_CDC1		1

uint32_t USBD_CDC_GetChannel(uint8_t epnum);
uint32_t USBD_CDC_GetChanFromIf(uint8_t ifnum);

const extern struct CFGDESC_ CfgDesc;
#endif /* __USB_DEV_CONFIG_H */
