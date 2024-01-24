/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_x-cube-ble1.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4yy.h"
#include "bf_reg.h"
#include "board.h"
#include "cmdproc.h"
#include "config.h"
#include "dcc_tx.h"
#include "dcc_auto.h"

#include <string.h>
#include "vl53l0x_api.h"
#include "sensor.h"
#include <limits.h>
#include "auxilary.h"
#include "algorithm.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/** }@} *//* defgroup ErrCode */

#define BSP_BP_PORT GPIOC
#define BSP_BP_PIN  GPIO_PIN_13
#define debug_printf    trace_printf
#define debug_printff    trace_printff
// #define debug_printf    uart_printf_light

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
extern VL53L0X_Dev_t VL53L0XDevs[2];
/**
 * Global ranging struct
 */
VL53L0X_RangingMeasurementData_t RangingMeasurementData[2];

int UseSensorsMask = (1 << XNUCLEO53L0A1_DEV_CENTER)
		| (1 << XNUCLEO53L0A1_DEV_LEFT);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// analog results
uint16_t adc_result[AC_NCH];
uint32_t adcavg[AC_NCH];	// averaging filters
uint16_t adcval[AC_NCH];	// final readouts

uint16_t isense_max, isenseq_mA;

uint8_t encval;

volatile uint32_t utimer;
uint32_t msec;

uint16_t signon_timer;
#define SIGNON_TOUT	1000u	// seconds

void SysTick_Handler(void)
{
	HAL_IncTick();
	if (utimer)
		--utimer;

	void run_every_ms(void);
	run_every_ms();

	static uint16_t tdiv;

	if (++tdiv == 1000u)
	{
		tdiv = 0;

		LED_PORT->BSRR = LED_MSK << 16 | (~LED_PORT->ODR & LED_MSK);
		LED_DUTY ^= LED_DIM	^ LED_FULL;

		if (signon_timer)
			--signon_timer;
	}
}

void HAL_Delay(uint32_t ms)
{
	for (utimer = ms; utimer;) ;
}

/*
 * Configure clock to PLL fed by HSE, 84 MHz, 48 MHz for USB,
 * both APBs at 42 MHz, timers at 84 MHz
 */
static void ClockConfig_F401(void)
{
	RCC->CR |= RCC_CR_HSESEL;
	while (!(RCC->CR & RCC_CR_HSERDY));
	RCC->PLLCFGR = (RCC->PLLCFGR & RCC_PLLCFGR_RSVD)
		| RCC_PLLCFGR_PLLSRC_HSE
		| RCC_PLLCFGR_PLLMV(HSE_VALUE / 1000000u)
		| RCC_PLLCFGR_PLLNV(336)	// 192 for F411 @ 96 MHz
		| RCC_PLLCFGR_PLLPV(4)		// 2 for F411 @ 96 MHz
		| RCC_PLLCFGR_PLLQV(7);		// 4 for F411 @ 96 MHz
	RCC->CR |= RCC_CR_PLLON;	//
	// set Flash speed
	FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_2WS;	// 1ws 30..64, 3 ws 90..100
	while (!(RCC->CR & RCC_CR_PLLRDY));

	RCC->CFGR = RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_SW_PLL;	// APB2, APB1 prescaler = 2
	//while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}



#define APB1_FREQ	(HCLK_FREQ / 2)
#define CON_BAUD	115200u

void hw_init(void)
{
	ClockConfig_F401();

	RCC->AHB1ENR = RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN
		| RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_DMA1EN;
//	RCC->AHB2ENR = RCC_AHB2ENR_OTGFSEN;

	RCC->APB1ENR = RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM5EN | RCC_APB1ENR_USART2EN;

	LED_TIM->PSC = HCLK_FREQ / LED_FREQ / LED_STEPS - 1;
	LED_TIM->ARR = LED_STEPS - 1;
	LED_TIM->CCR1 = LED_DIM;
	LED_TIM->CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;
	LED_TIM->BDTR = TIM_BDTR_MOE;
	LED_TIM->CCER = TIM_CCER_CC1E;
	//LED_TIM->DIER = TIM_DIER_CC1IE | TIM_DIER_UIE;
	LED_TIM->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

	DCC_TIM->CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE
			| TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;

	USART2->BRR = (APB1_FREQ + CON_BAUD / 2) / CON_BAUD;
	USART2->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_RXNEIE;

	GPIOA->AFR[0] = (union bf4_){
		.p0 = AFN_TIM3,	// TIM5CH1 - IN1B
		.p1 = AFN_TIM3,	// TIM5CH2 - IN2B
		.p2 = AFN_USART1_2,
		.p3 = AFN_USART1_2,
		.p5 = AFN_TIM1_2,	// TIM2CH1 - LED
	}.w;
	GPIOA->AFR[1] = (union bf4_){
		.p11 = AFN_USB,
		.p12 = AFN_USB,
	}.w;
	GPIOA->OSPEEDR |= (union bf2_){
		.p0 = GPIO_OSPEEDR_HI,
		.p1 = GPIO_OSPEEDR_HI,
		.p11 = GPIO_OSPEEDR_HI,
		.p12 = GPIO_OSPEEDR_HI	// USB
	}.w;
	GPIOA->MODER = (union bf2_){
		.p0 = GPIO_MODER_AF,	// TIM5CH1, CH2
		.p1 = GPIO_MODER_AF,
		.p2 = GPIO_MODER_AF,	// USART2
		.p3 = GPIO_MODER_AF,
		.p5 = GPIO_MODER_AF,
		.p11 = GPIO_MODER_AF,	// USB
		.p12 = GPIO_MODER_AF,	// USB
		.p13 = GPIO_MODER_AF,
		.p14 = GPIO_MODER_AF,
	}.w;
	GPIOA->PUPDR = GPIOA_PUPDR_SWD;

	GPIOC->MODER = (union bf2_){
		.p1 = GPIO_MODER_OUT,	// ENB
	}.w;

	NVIC_SetPriority(USART2_IRQn, 12);
	NVIC_EnableIRQ(USART2_IRQn);
}

int BSP_GetPushButton(void)
{
	GPIO_PinState state;
	state = HAL_GPIO_ReadPin(BSP_BP_PORT, BSP_BP_PIN);
	return state;
}

void OwnDemo(int UseSensorsMask)
{
	int status;
	int i;
	for (i = 0; i < 2; i++)
	{
		if (!VL53L0XDevs[i].Present)
			continue;

		status = VL53L0X_SetDeviceMode(&VL53L0XDevs[i],
		VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
		if (status == VL53L0X_ERROR_NONE)
			status = VL53L0X_StartMeasurement(&VL53L0XDevs[i]);
		if (status != VL53L0X_ERROR_NONE)
//			HandleError(2);
			while(1);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	int ExitWithLongPress;
	RangingConfig_e RangingConfig = HIGH_ACCURACY;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */
	hw_init(); // uart conflict // tim5 conflict // gpioa or instead of assign conflict // nadpisanie zegara?
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_I2C1_Init();
  MX_BlueNRG_MS_Init();
  /* USER CODE BEGIN 2 */

	// flash write unlock
	FLASH->KEYR = FLASH_FKEY1;
	FLASH->KEYR = FLASH_FKEY2;

	SysTick_Config(HCLK_FREQ / 1000u);
	NVIC_SetPriority(SysTick_IRQn, 11);	// must be higher than command processor priority

	// load config from Flash
	loadcfg();
	br_set_mode(cd.n.mode);
	// init H-bridge operation

	NVIC_SetPriority(DCC_PktAs_IRQn, 1);
	NVIC_EnableIRQ(DCC_PktAs_IRQn);

	/* Set VL53L0X API trace level */
//	VL53L0X_trace_config(NULL, TRACE_MODULE_NONE, TRACE_LEVEL_NONE, TRACE_FUNCTION_NONE); // No Trace
//	VL53L0X_trace_config(NULL,TRACE_MODULE_ALL, TRACE_LEVEL_ALL, TRACE_FUNCTION_ALL); // Full trace
	InitSensors(&hi2c1, RangingConfig);
	/* offset, xtalk calibrations below*/
//	uint32_t pOffsetMicroMeter[2] = {0};
//	uint32_t measured_distanceoffset = 20*10; //150mm
//
//	uint32_t measured_distance0 = 43*10; //mm
//	uint32_t measured_distance1 = 72*10; //mm
//	uint32_t pXTalkCompensationRateMegaCps[2] = {0};
//	volatile VL53L0X_Error err = VL53L0X_PerformXTalkCalibration(&VL53L0XDevs[0], measured_distance0, &pXTalkCompensationRateMegaCps[0]);
//	volatile VL53L0X_Error err0 = VL53L0X_PerformOffsetCalibration(&VL53L0XDevs[1], measured_distance1, &pOffsetMicroMeter[1]);
//	char s[10];
//	sprintf(s, "offset calib passed\r\n");
//	con_putstr(s);
//	HAL_Delay(5000);
//	volatile VL53L0X_Error err2 = VL53L0X_PerformXTalkCalibration(&VL53L0XDevs[1], measured_distance1, &pXTalkCompensationRateMegaCps[1]);
	OwnDemo(UseSensorsMask);
	HAL_TIM_Base_Start_IT(&htim10);
	HAL_TIM_Base_Start_IT(&htim11);
	// trace_printf("%d,%u,%d,%d,%d\n", VL53L0XDevs[i].Id, TimeStamp_Get(), RangingMeasurementData.RangeStatus, RangingMeasurementData.RangeMilliMeter, RangingMeasurementData.SignalRateRtnMegaCps);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
//		__WFI();
    /* USER CODE END WHILE */

  MX_BlueNRG_MS_Process();
    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 9999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 1049;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 9999;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 2099;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SENSOR_1_Pin|SENSOR_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR_1_Pin SENSOR_2_Pin */
  GPIO_InitStruct.Pin = SENSOR_1_Pin|SENSOR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim10)
	{
		for (int i = 0; i < 2; i++)
		{
			if (!VL53L0XDevs[i].Present)
				continue;
			uint8_t isReady = 0;
			VL53L0X_Error status = VL53L0X_GetMeasurementDataReady(
					&VL53L0XDevs[i], &isReady);
			if (status == VL53L0X_ERROR_NONE)
			{
				if (isReady)
				{
					status = VL53L0X_GetRangingMeasurementData(&VL53L0XDevs[i],
							&RangingMeasurementData[i]);

					if (status == VL53L0X_ERROR_NONE)
						status = VL53L0X_ClearInterruptMask(&VL53L0XDevs[i], 0);

					if (status == VL53L0X_ERROR_NONE)
						Sensor_SetNewRange(&VL53L0XDevs[i],
								&RangingMeasurementData[i]);

//					trace_printf("%d,%d,%d\n", VL53L0XDevs[i].Id, RangingMeasurementData[i].RangeStatus, RangingMeasurementData[i].RangeMilliMeter);
					// char tab[6];
					// sprintf(tab, "%d%d%d",VL53L0XDevs[i].Id, RangingMeasurementData[i].RangeStatus + 48, RangingMeasurementData[i].RangeMilliMeter);
					// debug_printff(tab, 10);
				} else
				{
					__NOP();
				}
			}
		}
	}
	else if (htim == &htim11)
	{
		const struct autostep_ *sp = &autopgm[curr_step];
		if(itof_flag)
		{
			uint8_t l = sp->locnum;
			loco[l].rev = sp->rev;
			loco[l].dspeed = sp->speed;
			loco[l].fun.w[0] = sp->f0_28;
			if (isInRange(sp->idtof))
			{
				loco[l].dspeed = calcStep(sp->idtof);
			}

		}
	}
}



// USART2 console ========================================================
// If command interpreter runs at the same priority as UART interrupt,
// the buffer must be big enough for the longest message (help text)
#define OBSIZE	2048
static char outbuf[OBSIZE];
static uint16_t obputidx;
static volatile uint16_t obgetidx;

void con_putchar(char c)
{
	uint16_t newputidx = (obputidx + 1) % OBSIZE;
	if (newputidx != obgetidx)
	{
		// buffer not full, put character
		outbuf[obputidx] = c;
		__disable_irq();
		obputidx = newputidx;
		USART2->CR1 |= USART_CR1_TXEIE;
		__enable_irq();
	}
}

void putstr(const char *s)
{
	while (*s)
		con_putchar(*s++);
}

_Bool process_input(char c);

void USART2_IRQHandler(void)
{
	uint32_t sr = USART2->SR & USART2->CR1 & (USART_CR1_RXNEIE | USART_CR1_TXEIE);	// filter only enabled interrupts

	if (sr & USART_SR_RXNE)
	{
		if (!signon_timer)
		{
			display_signon();
			USART2->DR;	// ignore first char
		}
		else if (process_input(USART2->DR))
			display_prompt();
		signon_timer = SIGNON_TOUT;
	}

	if (sr & USART_SR_TXE)
	{
		// tx ready, something to send
		uint8_t c = outbuf[obgetidx];
		if (++obgetidx == OBSIZE)
			obgetidx = 0;
		if (obgetidx == obputidx)	// buffer empty
			USART2->CR1 &= ~USART_CR1_TXEIE;	// disable further TX interrupts
		USART2->DR = c;	// clears TCIF
	}
}
//========================================================================
void pktdump(const struct sendpacket_ *p)
{

}

void hw_boot(void)
{

}

void adc_control(void)
{

}

void adc_process(void)
{

}

void adc_store_csense(void)
{

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
