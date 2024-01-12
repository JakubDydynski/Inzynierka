#include "stm32f4yy.h"

#define NUCLEO_L6206

#define	ONE_HALFPERIOD	58
#define ZERO_HALFPERIOD	(ONE_HALFPERIOD * 2)

// Selectrix pulse widths
#define	SX_ACTIVE	40
#define SX_INACTIVE	10

#ifdef NUCLEO_L6206
// F401 Nucleo
#define DEVNAME	"Nucleo DCC controller by gbm"

#define HCLK_FREQ	84000000u
#define SYSCLK_FREQ	HCLK_FREQ
#define TIMCLK_FREQ	HCLK_FREQ
#define PCLK_FREQ	(HCLK_FREQ/2)	// both peripheral buses at /2

#define LED_PORT	GPIOA
#define LED_BIT	5

#define LED_MSK	(1u << LED_BIT)
#define LED_TIM	TIM2
#define LED_DUTY	LED_TIM->CCR2

// Right-side bridge (CN10)
#define DCC_TIM		TIM5

#define DCC_TIM_IRQHandler	TIM5_IRQHandler
#define DCC_TIM_IRQn	TIM5_IRQn

#define DCC_DUTY	TIM5->CCR1
#define DCC_DUTY2	TIM5->CCR2

#define AN_TIM	DCC_TIM
#define AN_FWD_DUTY	DCC_DUTY

#define EN1_PORT	GPIOC
#define EN1_BIT		1

// dummy!!!
#define NACK_PORT	GPIOC
#define NACK_BIT	2

#define FLASH_PAGE_SIZE	0x4000
#define CFGADDR	0x0800C000
#define ARUNADDR	0x0800D000	// autorun storage
#define CFG_SINGLE_PAGE

//#define CSENSOR

#else
#define CDC_CHANNELS	2
#define	CDC_DATA_PACKET_SIZE	64

#define BEMF
//#define CSENSOR
//#define INA219

#define FLASH_PAGE_SIZE	1024u
#define CFGADDR	0x0800Fc00
#define ARUNADDR	0x0800F800	// autorun storage
//#define CFGPAGE	31

#define SYSCLK_FREQ	72000000u
#define HCLK_FREQ	72000000u
#define PCLK_FREQ	(HCLK_FREQ/2)	// both peripheral buses at /2
#ifdef COREBOARD
#define DEVNAME	"F103 CoreBoard DCC controller by BlueDigital"
#include "Boards/stm32f103coreboard.h"
// Button at PA8, LED at PA1, defined in board def file
#define DCC_DUTY	TIM1->CCR2

// software-driven I2C for EEPROM
#define SI2C_TIM	TIM4
#define SI2C_TIM_IRQn	TIM4_IRQn
#define SI2C_TIM_IRQHandler	TIM4_IRQHandler
#define SI2C_FREQ 50000u
#define SSCL_PORT	GPIOB
#define SSCL_BIT	4u
#define SSDA_PORT	GPIOB
#define SSDA_BIT	6u

#define SSCL_MSK	(1u << SSCL_BIT)
#define SSDA_MSK	(1u << SSDA_BIT)
#else	// BluePill
#define DEVNAME	"F103 BluePill DCC controller by BlueDigital"
//#include "stm32f103coreboard.h"
#define LED_PORT	GPIOC
#define LED_BIT		13
#define LED_TIM	TIM2
#define LED_DUTY	LED_TIM->CCR2

#define LED_TIM_IRQn	TIM2_IRQn
#define LED_TIM_IRQHandler	TIM2_IRQHandler

#define DCC_DUTY	TIM1->CCR2
//#define DCC_CCRA	CCR2
#endif	// COREBOARD/BluePill
#endif	// NUCLEO_L6206/F103

#define SYSTICK_FREQ	1000u

#define RS485_BAUD	115200	// ESP8266

#include "dccctrl_config.h"

#define SHORT_CURRENT_mA	3000u
#define SHORT_TIMEOUT_ms	50u

#define ESP_UART	USART1

#if 0

#ifdef DCON1a
// supply voltage meas on PA0
// RS485 on USART2 PA2,3 , h/w DE control PA1
#define UART2_BAUD	RS485_BAUD	// ESP8266, RS485
#define	RS485_UART	USART2
#define	RS485_UART_IRQn	USART2_IRQn
#define	RS485_UART_IRQHandler	USART2_IRQHandler
#define RS485_DE_BIT	1

#define AN_TIM	TIM1
#define AN_FWD_DUTY	TIM1->CCR3	// todo!!!
//#define AN_REV_DUTY	TIM1->CCR3	// todo!!!

// ESP8266, WS2812 on USART1 PA9, 10
//#define UART1_BAUD	115200	// BLE

// nRF24L01 on MOSI1, PB3..6, CE-PB7

// LED on PH3/BOOT, active high
#define LED_MSK		(1u << LED_BIT)

// LED control via DMA - TIM2CH1 - DMA1Ch5, TIM2UP - DMA1Ch2, DMA sel 4
//HB current sense on PB0,1
// DCC -ACK on PB12

// TIM1CH3N
//#define EN2_PORT	GPIOB
//#define EN2_PIN		15
// TIM1CH1
//#define EN1_PORT	GPIOA
//#define EN1_PIN		8

#define NACK_PORT	GPIOB
#define NACK_BIT	12

//#define ADCH_VSUP	5u
#define ADCH_BEMF	0u	// PA0 (Coreboard has LED on PA1)
#define ADCH_POT	2	// PA2
#define ADCH_IS1	8u	// PB0
#define ADCH_IS2	9u	// PB1
//#define ADCH_TSEN	16u
#define ADCH_VREF	17u

#define	BEMF_MAX	12000u
#define RBH	3		// 10k
#define	RBL	1		// RH in parallel to RL, RL = 1/2 RH (2 RH in parallel)
#define	BEMF_SCALE	(RBH + RBL) / RBL	// no parentheses!!
#define AVGSHIFT	2
extern uint16_t adc_result[AC_NCH];
extern uint32_t adcavg[AC_NCH];	// averaging filters
extern uint16_t adcval[AC_NCH];	// final readouts

#define RCCOUT_DMACh	DMA1_Channelx	// DMA channel for Railcom cutout control
	
#define	ADC1_DMACh	DMA1_Channel1
//#define ADC_EXTSEL_TIM15_TRGO	14

#else
#define UART1_BAUD	RS485_BAUD	// ESP8266, RS485
#define UART2_BAUD	115200	// BLE

#define	RS485_UART	USART1
#define	RS485_UART_IRQn	USART1_IRQn
#define	RS485_UART_IRQHandler	USART1_IRQHandler

#define EN1_PORT	GPIOB
#define EN1_PIN		15
#define EN2_PORT	GPIOA
#define EN2_PIN		8

#define RS485_DE_BIT	0
#endif

#define DCC_TIM		TIM1

#define DCC_TIM_IRQHandler	TIM1_UP_IRQHandler
#define DCC_TIM_IRQn	TIM1_UP_IRQn

#endif

//#define RS485_TOUT_TIM	TIM6
//#define RS485_PKTTOUT	5000	// us

#define AN_DEF_STEPS	100u
#define AN_DEF_FREQ	300u

#define RS485_DE_PORT	GPIOA
#define RS485_DE_MASK	(1u << RS485_DE_BIT)

#define EN1_MSK	(1u << EN1_BIT)
#define EN2_MSK	(1u << EN2_BIT)
#define NACK_MSK	(1u << NACK_BIT)

// Software interrupt
#define DCC_PktAs_IRQHandler	USART6_IRQHandler
#define DCC_PktAs_IRQn	USART6_IRQn

#define SX_PktAs_IRQHandler	RCC_IRQHandler
#define SX_PktAs_IRQn	RCC_IRQn

//#define Frame_IRQn	CRS_IRQn
//#define Frame_IRQHandler	CRS_IRQHandler

#define Fdump_IRQn	TIM1_BRK_IRQn
#define Fdump_IRQHandler	TIM1_BRK_IRQHandler

#define USB_IRQ_PRI	12

//enum adch_ {AC_VREF, AC_BEMF, AC_POT, AC_NCH};
enum adch_ {AC_VREF, AC_TSEN, AC_VBEMF, AC_IS1, AC_IS2, AC_VSUP, AC_POT, AC_NCH};
extern uint16_t adc_result[AC_NCH];
extern uint32_t adcavg[AC_NCH];	// averaging filters
extern uint16_t adcval[AC_NCH];	// final readouts

extern uint16_t isense_max, isenseq_mA;

#define LED_STEPS	50
#define LED_FREQ	400
#define LED_DIM		2
#define LED_FULL	(LED_STEPS - 1)	// in case of DMA control

extern uint16_t led_timer;

void hw_boot(void);
void adc_process(void);
void adc_store_csense();
