/*
	hardware-independent, periodic execution and timing for DCC controller
	gbm 10'2023
	
	SysTick interrupt must have priority higher than command interpreter
	to implement automatic function deactivation delay
*/

#include "board.h"
#include "dcc_tx.h"
#include "dcc_auto.h"

#ifdef INA219
#include "ina219.h"

const char *const csname[] = {"None", "INA219", "INA3221"};
#endif


#define DEMO_PERIOD	2000

// LED control
// 50 steps, led control run with 10 ms interval
// setting led_timer to 100 results in LED turning on, then off in 1 second
uint16_t led_timer;

static void led_control(void)
{
	static uint8_t led_target = LED_DIM;
	
	if (led_timer)
		led_target = --led_timer == 0 ? LED_DIM : LED_FULL;
	uint16_t led_curr = LED_DUTY;
	if (led_curr != led_target)
		LED_DUTY = led_curr < led_target ? led_curr + 2 : led_curr - 1;
}

volatile uint16_t seccnt;

static void run_every_10ms(void)
{
#ifdef USE_ADC
	void adc_control(void);

	adc_control();
#endif
	led_control();

	// sensor data acquisition

	static uint8_t tdiv;
	if (++tdiv == 100u)
	{
		// 1 s elapsed
		tdiv = 0;
		seccnt++;
		autorun();
	}
}

void run_every_ms(void)
{
	adc_process();
#ifdef INA219
	//static uint8_t i2cdiv;
	
	//if (++i2cdiv == 3)
	{
		//i2cdiv = 0;
		void i2c_acquire(void);
		i2c_acquire();
	}

	if (ina219.current_100uA >= 600 || adcval[AC_IS1] >= isenseq_mA + 60)
	{
		dccs.ack = 1;
		led_timer = LED_FREQ;
	}
	else if (dccs.ack)
		dccs.ack_released = 1;
	
	static uint8_t short_timer;
	static _Bool reversed;
	// short circuit handling, incl. loop reversal
	if (ina219.current_100uA >= SHORT_CURRENT_mA * 10)
	{
#if 0	// reversing loop
		static _Bool loop_rev;
		if (!reversed)
		{
			// reverse once
			DCC_TIM->CCER ^= TIM_CCER_CC2P | TIM_CCER_CC2NP;
			loop_rev ^= 1;
			reversed = 1;
		}
#endif
		if (++short_timer == SHORT_TIMEOUT_ms)
		{
			// turn off both DCC outputs
			// todo: analog!
			//DCC_TIM->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE);
		}
	}
	else
	{
		if (short_timer >= SHORT_TIMEOUT_ms)
		{
			// turned off because of short circuit
			if (++short_timer == SHORT_TIMEOUT_ms * 2)
			{
				// restore operation
				//DCC_TIM->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE;
				short_timer = 0;
			}
		}
		else
		{
			// normal operation
			short_timer = 0;
			reversed = 0;
		}
	}
	
#else
	// DCC Ack detector logic input
	if (~NACK_PORT->IDR & NACK_MSK)
	{
		dccs.ack = 1;
		led_timer = LED_FREQ;
	}
	else if (dccs.ack)
		dccs.ack_released = 1;
	
#endif
	
#if 0
	if (ctrl_mode == CM_DEMO1)
	{
		static uint16_t ctimer;
		
		if (++ctimer == DEMO_PERIOD)
		{
			ctimer = 0;
			static uint8_t bright_lamp;
			loco[bright_lamp].dspeed = 5;
			for (uint8_t i = 1; i < NDEVICES; i++)
			{
				uint8_t li = (bright_lamp + i) % NDEVICES;
				if (cd.n.dev[li].flags.w)
				{
					bright_lamp = li;
					loco[bright_lamp].dspeed = 255;
					break;
				}
			}
		}
	}
#endif
	static uint8_t tdiv;
	if (++tdiv == 10)
	{
		tdiv = 0;
		void run_every_10ms(void);
		run_every_10ms();
	}
}

