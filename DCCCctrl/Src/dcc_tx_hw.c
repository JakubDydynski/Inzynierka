/*
	hw-dependent H-bridge interafce

 DCC transmitter with programming
 Selectrix transmitter
 gbm 04'2020-2023
 04'23 support for X-Nucleo L6206 on Nucleo-F401 or -L476 - two timer channels used for DCC
*/

#include "board.h"
#include "dcc_tx.h"
#include "config.h"
#include <string.h>

const char *const hbname[] = {"BTN/IFX", "DRV8871", "TB6612", "L6206"};

// DCC timing
#define	ONE_HALFPERIOD	58
#define ZERO_HALFPERIOD	(ONE_HALFPERIOD * 2)

// Selectrix pulse widths
#define	SX_ACTIVE	40
#define SX_INACTIVE	10

#define NEWDCCTX	// simple DCC Tx control via prescaler
#define NEWDCC_STEPS	2u
#define TIM_us_DIV	(TIMCLK_FREQ / 1000000u)
#define NEWDCC_PRE1	(TIM_us_DIV * ONE_HALFPERIOD * 2u / NEWDCC_STEPS)
#define NEWDCC_PRE0	(TIM_us_DIV * ZERO_HALFPERIOD * 2u / NEWDCC_STEPS)
// Sender ================================================================
// Called by DCC Tx control to set the next bit value
void prepare_DCC_bit(_Bool bit)
{
#ifdef NEWDCCTX
	DCC_TIM->PSC = (bit ? NEWDCC_PRE1 : NEWDCC_PRE0) - 1;
#else
	uint32_t halfperiod = bit ? ONE_HALFPERIOD : ZERO_HALFPERIOD;
	
	DCC_TIM->ARR = 2 * halfperiod - 1;
	DCC_DUTY = halfperiod;
#ifdef DCC_CCRB
	DCC_TIM->DCC_CCRB = halfperiod;
#endif
#endif
}

// called by SX Tx control
void prepare_SX_bit(_Bool bit)
{
	if (bit)
		DCC_TIM->CCER ^= TIM_CCER_CC1E | TIM_CCER_CC1NE
			| TIM_CCER_CC2E	| TIM_CCER_CC2NE;	// toggle active out 
}

// H-bridge period interrupt ==============================================
void DCC_TIM_IRQHandler(void)
{
	//if (DCC_TIM1->SR & TIM_SR_UIF)
	{
		DCC_TIM->SR = ~TIM_SR_UIF;	// the only source
		// store adc current sense readouts
		adc_store_csense();
		dcc_tx_next_bit();
	}
}
// bridge control routines
struct anstate_ ans = {
	.freq = AN_DEF_FREQ,
	.steps = AN_DEF_STEPS
};

static void br_init_analog(void)
{
	// set forward dir, speed 0
	// DCC TIM duty is PH2, 1-duty is PH1
	DCC_TIM->PSC = TIMCLK_FREQ / ans.steps / ans.freq - 1;
	DCC_TIM->ARR = ans.steps - 1;
	AN_TIM->PSC = TIMCLK_FREQ / ans.steps / ans.freq - 1;
	AN_TIM->ARR = ans.steps + 2;

	ans.active = 1;
}

/*========================================================================
	H-bridge control setup 
	the explicit argument allows for turning the bridge off without changing the configured op mode

	Init for Selectrix operation - 50 us period, 40 us active pulse on one of outputs.
	Interrupt at the end of active pulse to select active output for the next period
*/

void br_set_mode(enum ctrlmode_ mode)
{
	NVIC_DisableIRQ(DCC_TIM_IRQn);	// DCC
	NVIC_DisableIRQ(TIM1_CC_IRQn);	// analog BEMF & SX
	ans.active = 0;
	DCC_TIM->CCER = 0;	// all outputs low
#ifdef NUCLEO_L6206
	EN1_PORT->BSRR = EN1_MSK << 16;
#endif
	// With OSSR set, the disabled output of complementary pair outputs inactive state
	DCC_TIM->BDTR = TIM_BDTR_MOE | TIM_BDTR_OSSR;	// default - output inactive state on inactive complementary outputs
	// for non-analog operation
//	DCC_TIM->DIER = TIM_DIER_UIE | TIM_DIER_CC2IE;	// tak bylo na F103
	DCC_TIM->DIER = TIM_DIER_UIE;
	
	switch (mode)
	{
	case CM_OFF:
	default:
		break;
	case CM_ANCOAST:	// one terminal at GND, other V+ or off
		br_init_analog();
		switch (cd.n.hbtype)
		{
		case HB_IFX:
		default:
			// PWM on ENx, INx state depends on direction
			DCC_DUTY = ans.dir_rev ? 0xffff : 0;
			AN_FWD_DUTY = 0;
			if (ans.dir_rev)
				DCC_TIM->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE	// PWM out on IN1
					| TIM_CCER_CC3E | TIM_CCER_CC3NP	// CC3N output high
					| TIM_CCER_CC4E;	// for DCON1a EN2 control
			else
				DCC_TIM->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE	// PWM out on IN1
					| TIM_CCER_CC3P | TIM_CCER_CC3NE	// CC3 output high
					| TIM_CCER_CC4E;	// for DCON1a EN2 control
			break;
			
		case HB_DRV8871:
			DCC_DUTY = 0;
			if (ans.dir_rev)
				DCC_TIM->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;	// PWM out on IN1
			else
				DCC_TIM->CCER = TIM_CCER_CC1NE | TIM_CCER_CC2NE;	// PWM out on IN2
			break;
			
		case HB_TB6612:
			DCC_DUTY = 0;
			AN_FWD_DUTY = 0xffff;	// TB6612 PWM input HIGH
			if (ans.dir_rev)	// one IN Low, the other - PWM
				DCC_TIM->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;	// PWM out on IN1
			else
				DCC_TIM->CCER = TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3E;	// PWM out on IN2
			// BEMF experimental
			DCC_TIM->DIER = TIM_DIER_CC1IE;
			DCC_TIM->CCR1 = ans.steps + 1;
			DCC_TIM->SR = 0;
			NVIC_EnableIRQ(TIM1_CC_IRQn);
			break;
		}
		break;
		
	case CM_ANBRAKE:	// one terminal at GND, other V+ or GND
		br_init_analog();
		switch (cd.n.hbtype)
		{
		case HB_IFX:
		default:
			DCC_DUTY = 0;	// controls analog op
			AN_FWD_DUTY = 0xffff;	// EN1/2 high
#ifdef AN_REV_DUTY
			AN_REV_DUTY = 0xffff;
#endif
			// PWM on one of INx
			DCC_TIM->CCER = ans.dir_rev
				? TIM_CCER_CC1E | TIM_CCER_CC2E	// PWM out on IN1, IN2 = 0
					| TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC3NP	// CC3N output same polarity as CC3
					| TIM_CCER_CC4E	// for DCON1a EN2 control
				: TIM_CCER_CC1NE | TIM_CCER_CC2NE	// PWM out on IN2, IN1 = 0
					| TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC3NP	// CC3N output same polarity as CC3
					| TIM_CCER_CC4E;	// for DCON1a EN2 control
			break;
				
		case HB_DRV8871:
			DCC_DUTY = 0;
			if (ans.dir_rev)
				// IN1 high, IN2 -DUTY
				DCC_TIM->CCER = TIM_CCER_CC1NE | TIM_CCER_CC1NP | TIM_CCER_CC2NE | TIM_CCER_CC2NP;	// PWM out on IN1
			else	// IN2 high, IN1 -DUTY
				DCC_TIM->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC2E | TIM_CCER_CC2P;	// PWM out on IN1
			break;
			
		case HB_TB6612:
			DCC_DUTY = ans.dir_rev ? 0xffff : 0;
			AN_FWD_DUTY = 0;
			DCC_TIM->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE
				| TIM_CCER_CC2E | TIM_CCER_CC2NE	// PWM out on EN1/EN2/PWM
				| TIM_CCER_CC3E;	// analog pwm out
			break;
		}
		break;

	case CM_DCC:
		// same for all 3 bridge types, IN1/2 = DCC, EN1/2 = high
#ifdef NEWDCCTX
		DCC_DUTY = NEWDCC_STEPS / 2;	// CCR2 is DCC duty/controls INx, CCR3 controls EN 
		DCC_TIM->ARR = NEWDCC_STEPS - 1;
		DCC_TIM->PSC = NEWDCC_PRE1 - 1;
#else
		DCC_TIM->PSC = TIMCLK_FREQ / 1000000 - 1;
		DCC_TIM->ARR = ONE_HALFPERIOD * 2 - 1;
		DCC_DUTY = ONE_HALFPERIOD;	// CCR2 is DCC duty/controls INx, CCR3 controls EN 
#endif
#ifdef NUCLEO_L6206
		// complementary outputs from TIM3CH1 and TIM3CH2
		DCC_DUTY2 = NEWDCC_STEPS / 2;	// CCR2 is DCC duty/controls INx, CCR3 controls EN 
		DCC_TIM->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P;
		DCC_TIM->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
		// enable high by port setting
		EN1_PORT->BSRR = EN1_MSK;
#else
		// timer-native complementary outputs; IN1 & IN2 - DCC, EN1/EN2 high
		DCC_TIM->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE
			| TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC3NP	// CC3N output same polarity as CC3
			| TIM_CCER_CC4E;	// for DCON1a EN2 control
		// set inactive output states for breakout - both INs 0, EN 1
		AN_FWD_DUTY = 0xffff;
	#ifdef AN_REV_DUTY
		AN_REV_DUTY = 0xffff;
	#endif
#endif
		NVIC_EnableIRQ(DCC_TIM_IRQn);
		break;

	case CM_SX:
		DCC_TIM->BDTR = TIM_BDTR_MOE;	// turn off OSSR for Selectrix only
		// same for all 3 bridge types, IN1/2 = DCC, EN1/2 = high
		DCC_TIM->PSC = TIM_us_DIV - 1;
		DCC_TIM->ARR = SX_ACTIVE + SX_INACTIVE - 1;
		DCC_DUTY = SX_ACTIVE; 
		DCC_TIM->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E	// one of IN outputs active, the other is 0
			| TIM_CCER_CC3E | TIM_CCER_CC3NE | TIM_CCER_CC3NP	// CC3N output same polarity as CC3
			| TIM_CCER_CC4E;	// for DCON1a EN2 control
		AN_FWD_DUTY = 0xffff;
	#ifdef AN_REV_DUTY
		AN_REV_DUTY = 0xffff;
	#endif
		sx_start();
		NVIC_EnableIRQ(TIM1_CC_IRQn);
		break;
	}
}

void br_set_andir(_Bool fwd)
{
	if (ans.dir_rev != fwd)
	{
		ans.dir_rev = fwd;
		ans.duty = 0;
		br_set_mode((enum ctrlmode_) cd.n.mode);
	}
}

void br_set_anduty(uint16_t duty)
{
	if (duty > ans.steps)
		duty = ans.steps;
	switch (cd.n.hbtype)
	{
	case HB_IFX:
	default:
		if (cd.n.mode == CM_ANBRAKE)
			DCC_DUTY = duty;
		else
			AN_FWD_DUTY = duty;
		break;
	case HB_DRV8871:
		DCC_DUTY = duty;
		break;
	case HB_TB6612:
		if (cd.n.mode == CM_ANBRAKE)
			AN_FWD_DUTY = duty;
		else
			DCC_DUTY = duty;
	}
	ans.duty = duty;
}

void br_cutout_start(void)
{
	// setup DMA for BDTR MOE bit clearing
}

void br_cutout_end(void)
{
}

/*
	IBT-2, SR - 1k to ground
	pin
	1, 2	IN1,2 - output level control
	3, 4	INH1, 2 - low-sleep, high - active
	5, 6	IS, 1k to GND
	7 - Vcc, 8 - GND
*/

#ifdef	USE_BEMF

// BEMF-driven speed control in analog mode ==============================
#define	DRIVE_MAGN	4096
#define INTEGRAL_MIN	(-100 * 4096)
#define INTEGRAL_MAX	(100 * 4096)

struct pid_ {
	int32_t target_adc;
	int32_t current_adc;
	int32_t error;
	int32_t prev_adc;
	int32_t integral;
	int32_t derivative;
	int32_t drive;
	int32_t bias;
	uint8_t kp_lshift;
	uint8_t ki_rshift;
	uint8_t	kd_lshift;
} pid = {
	.kp_lshift = 0,
	.ki_rshift = 6,
	.kd_lshift = 0
};

#endif

// TODO: make portable!
// Analog speed control by BEMF, Selectrix next bit loading
void TIM1_CC_IRQHandler(void)
{
	// may use any CC channel, so clear all CCIF flags
	TIM1->SR = ~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);
#define FSH 1
	switch (cd.n.mode)
	{
	case CM_ANCOAST:
#ifdef USE_BEMF
		if (ans.bemf_on)
		{
			static uint16_t bemfavg;
			bemfavg = bemfavg - (bemfavg >> FSH) + adc_result[AC_BEMF];

#if 1	// PID			
			pid.target_adc = ans.speed * (BEMF_MAX / BEMF_SCALE) * 4096 / 100 / adcval[AC_VREF];
			pid.current_adc = bemfavg >> FSH;
			pid.error = pid.target_adc - pid.current_adc;
			
			pid.integral += pid.error;
			if (pid.integral < (-DRIVE_MAGN << pid.ki_rshift))
				pid.integral = -DRIVE_MAGN << pid.ki_rshift;
			else if (pid.integral > (DRIVE_MAGN << pid.ki_rshift))
				pid.integral = DRIVE_MAGN << pid.ki_rshift;
			
			int32_t pid_iTerm = pid.integral >>= pid.ki_rshift;
			pid.derivative = pid.prev_adc - pid.current_adc;
			
			pid.drive = (pid.error << pid.kp_lshift)
				+ pid_iTerm
				+ (pid.derivative << pid.kd_lshift);
			//pid.drive += pid.bias;
			
			if (pid.drive < 0)
				pid.drive = 0;
			else if (pid.drive > 4096)
				pid.drive = 4096;
			ans.duty = pid.drive * 100 / 4096;
			
			// pwm follower by gbm
			if (pid_iTerm)
			{
				if (pid_iTerm > 0)
					++pid.bias;
				else
					--pid.bias;
			}
			
			pid.prev_adc = pid.current_adc;
			
#else	// no PID
			ans.bemf_mV = (Vref_mV * bemfavg * BEMF_SCALE / 4096u) >> FSH;
			uint16_t target_bemf_mV = ans.speed * BEMF_MAX / 100;
			if (ans.bemf_mV < target_bemf_mV)
			{
				if (ans.duty < ans.steps)
					++ans.duty;
			}
			else
			{
				if (ans.duty > ans.speed * 3/4)
					--ans.duty;
			}
#endif	// PID/no PID
			br_set_anduty(ans.duty);
		}
#endif	// USE_BEMF
		break;
		
	case CM_SX:
		sx_tx_next_bit();
		break;
	default:
		;
	}
}
