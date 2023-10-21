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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include "X-NUCLEO-53L0A1.h"
#include "vl53l0x_api.h"
#include <limits.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
	LONG_RANGE 		= 0, /*!< Long range mode */
	HIGH_SPEED 		= 1, /*!< High speed mode */
	HIGH_ACCURACY	= 2, /*!< High accuracy mode */
} RangingConfig_e;
typedef enum {
	RANGE_VALUE 	= 0, /*!< Range displayed in cm */
	BAR_GRAPH 		= 1, /*!< Range displayed as a bar graph : one bar per sensor */
} DemoMode_e;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/**
 * @defgroup Configuration Static configuration
 * @{
 */
#define HAVE_ALARM_DEMO 0

/** Time the initial 53L0 message is shown at power up */
#define WelcomeTime 660

/** Time the initial 53L0 message is shown at power up */
#define ModeChangeDispTime  500

/**
 * Time considered as  a "long push" on push button
 */
#define PressBPSwicthTime   1000

/** @}  */ /* config group */

#ifndef MIN
#   define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC

#ifndef ARRAY_SIZE
#   define ARRAY_SIZE(x) (sizeof((x))/sizeof((x)[0]))
#endif

/**
 * @defgroup ErrCode Errors code shown on display
 * @{
 */
#define ERR_DETECT             -1
#define ERR_DEMO_RANGE_ONE     1
#define ERR_DEMO_RANGE_MULTI   2

/** }@} */ /* defgroup ErrCode */

#define BSP_BP_PORT GPIOC
#define BSP_BP_PIN  GPIO_PIN_13
#define debug_printf    trace_printf

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/**
 * Global ranging struct
 */
VL53L0X_RangingMeasurementData_t RangingMeasurementData;


int UseSensorsMask = (1 << XNUCLEO53L0A1_DEV_CENTER) | (1 << XNUCLEO53L0A1_DEV_LEFT);
/** leaky factor for filtered range
 *
 * r(n) = averaged_r(n-1)*leaky +r(n)(1-leaky)
 *
 * */
int LeakyFactorFix8 = (int)( 0.6 *256);
/** How many device detect set by @a DetectSensors()*/
int nDevPresent=0;
/** bit is index in VL53L0XDevs that is not necessary the dev id of the BSP */
int nDevMask;

VL53L0X_Dev_t VL53L0XDevs[]={
        {.Id=XNUCLEO53L0A1_DEV_LEFT, .DevLetter='l', .I2cHandle=&hi2c3, .I2cDevAddr=0x52},
        {.Id=XNUCLEO53L0A1_DEV_CENTER, .DevLetter='c', .I2cHandle=&hi2c3, .I2cDevAddr=0x52},
        {.Id=XNUCLEO53L0A1_DEV_RIGHT, .DevLetter='r', .I2cHandle=&hi2c3, .I2cDevAddr=0x52},
};

/** range low (and high) in @a RangeToLetter()
 *
 * used for displaying  multiple sensor as bar graph
 */
int RangeLow=100;

/** range medium in @a RangeToLetter()
 *
 * used for displaying  multiple sensor as bar graph
 */
int RangeMedium=300;

char WelcomeMsg[]="Hi I am Ranging VL53L0X mcu " MCU_NAME "\n";

#if HAVE_ALARM_DEMO
volatile int IntrCount;
volatile int LastIntrPin;
volatile int LastIntrId;
volatile int IntrCounts[3];
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM10_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */
static void InitSensors(void);
static int ResetId(int DevNo, GPIO_PinState state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void TimeStamp_Reset(){
    HAL_TIM_Base_Start(&htim5);
    htim5.Instance->CNT=0;
}

uint32_t TimeStamp_Get(){
    return htim5.Instance->CNT;
}

int BSP_GetPushButton(void){
    GPIO_PinState state ;
    state = HAL_GPIO_ReadPin(BSP_BP_PORT, BSP_BP_PIN);
    return state;
}

/**
 * When button is already pressed it waits for user to release it.
 * if button remains pressed for a given time it returns true.
 * This is used to detect mode switch by long press on blue Push Button
 *
 * As soon as time is elapsed -rb- is displayed to let user know the mode
 * switch is taken into account
 *
 * @return True if button remains pressed more than specified time
 */
int PusbButton_WaitUnPress(void){
    uint32_t TimeStarted;
    TimeStarted = HAL_GetTick();
    while( !BSP_GetPushButton() ){ ; /* debounce */
        if(HAL_GetTick()- TimeStarted> PressBPSwicthTime){
            XNUCLEO53L0A1_SetDisplayString (" rb ");
        }
    }
    return  HAL_GetTick() - TimeStarted>PressBPSwicthTime;

}

#if HAVE_ALARM_DEMO
/**
 * Interrupt handler called each time an interrupt is produced by the ranging sensor (in ALARM mode)
 * @param err
 */
void VL53L0A1_EXTI_Callback(int DevNo, int GPIO_Pin){
    IntrCount++;
    LastIntrPin=GPIO_Pin;
    LastIntrId=DevNo;

    if( DevNo< ARRAY_SIZE(IntrCounts)  ){
        IntrCounts[DevNo]++;
    }
}
#endif


/**
 * Handle Error
 *
 * Set err on display and loop forever
 * @param err Error case code
 */
void HandleError(int err){
    char msg[16];
    sprintf(msg,"Er%d", err);
    XNUCLEO53L0A1_SetDisplayString(msg);
    while(1){};
}

/**
 *  Setup all detected sensors for single shot mode and setup ranging configuration
 */
void SetupSingleShot(RangingConfig_e rangingConfig){
    int i;
    int status;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
	uint8_t isApertureSpads;
	FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25*65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18*65536);
	uint32_t timingBudget = 33000;
	uint8_t preRangeVcselPeriod = 14;
	uint8_t finalRangeVcselPeriod = 10;

    for( i=0; i<3; i++){
        if( VL53L0XDevs[i].Present){
            status=VL53L0X_StaticInit(&VL53L0XDevs[i]);
            if( status ){
                debug_printf("VL53L0X_StaticInit %d failed\n",i);
            }

            status = VL53L0X_PerformRefCalibration(&VL53L0XDevs[i], &VhvSettings, &PhaseCal);
			if( status ){
			   debug_printf("VL53L0X_PerformRefCalibration failed\n");
			}

			status = VL53L0X_PerformRefSpadManagement(&VL53L0XDevs[i], &refSpadCount, &isApertureSpads);
			if( status ){
			   debug_printf("VL53L0X_PerformRefSpadManagement failed\n");
			}

            status = VL53L0X_SetDeviceMode(&VL53L0XDevs[i], VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
            if( status ){
               debug_printf("VL53L0X_SetDeviceMode failed\n");
            }

            status = VL53L0X_SetLimitCheckEnable(&VL53L0XDevs[i], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
			if( status ){
			   debug_printf("VL53L0X_SetLimitCheckEnable failed\n");
			}

			status = VL53L0X_SetLimitCheckEnable(&VL53L0XDevs[i], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit
			if( status ){
			   debug_printf("VL53L0X_SetLimitCheckEnable failed\n");
			}
			/* Ranging configuration */
            switch(rangingConfig) {
            case LONG_RANGE:
            	signalLimit = (FixPoint1616_t)(0.1*65536);
            	sigmaLimit = (FixPoint1616_t)(60*65536);
            	timingBudget = 33000;
            	preRangeVcselPeriod = 18;
            	finalRangeVcselPeriod = 14;
            	break;
            case HIGH_ACCURACY:
				signalLimit = (FixPoint1616_t)(0.25*65536);
				sigmaLimit = (FixPoint1616_t)(18*65536);
				timingBudget = 200000;
				preRangeVcselPeriod = 14;
				finalRangeVcselPeriod = 10;
				break;
            case HIGH_SPEED:
				signalLimit = (FixPoint1616_t)(0.25*65536);
				sigmaLimit = (FixPoint1616_t)(32*65536);
				timingBudget = 20000;
				preRangeVcselPeriod = 14;
				finalRangeVcselPeriod = 10;
				break;
            default:
            	debug_printf("Not Supported");
            }

            status = VL53L0X_SetLimitCheckValue(&VL53L0XDevs[i],  VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
			if( status ){
			   debug_printf("VL53L0X_SetLimitCheckValue failed\n");
			}

			status = VL53L0X_SetLimitCheckValue(&VL53L0XDevs[i],  VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
			if( status ){
			   debug_printf("VL53L0X_SetLimitCheckValue failed\n");
			}

            status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&VL53L0XDevs[i],  timingBudget);
            if( status ){
               debug_printf("VL53L0X_SetMeasurementTimingBudgetMicroSeconds failed\n");
            }

            status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDevs[i],  VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
			if( status ){
			   debug_printf("VL53L0X_SetVcselPulsePeriod failed\n");
			}

            status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDevs[i],  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);
			if( status ){
			   debug_printf("VL53L0X_SetVcselPulsePeriod failed\n");
			}

			status = VL53L0X_PerformRefCalibration(&VL53L0XDevs[i], &VhvSettings, &PhaseCal);
			if( status ){
			   debug_printf("VL53L0X_PerformRefCalibration failed\n");
			}

            VL53L0XDevs[i].LeakyFirst=1;
        }
    }
}

/* Store new ranging data into the device structure, apply leaky integrator if needed */
void Sensor_SetNewRange(VL53L0X_Dev_t *pDev, VL53L0X_RangingMeasurementData_t *pRange){
    if( pRange->RangeStatus == 0 ){
        if( pDev->LeakyFirst ){
            pDev->LeakyFirst = 0;
            pDev->LeakyRange = pRange->RangeMilliMeter;
        }
        else{
            pDev->LeakyRange = (pDev->LeakyRange*LeakyFactorFix8 + (256-LeakyFactorFix8)*pRange->RangeMilliMeter)>>8;
        }
    }
    else{
        pDev->LeakyFirst = 1;
    }
}

/**
 * Implement the ranging demo with all modes managed through the blue button (short and long press)
 * This function implements a while loop until the blue button is pressed
 * @param UseSensorsMask Mask of any sensors to use if not only one present
 * @param rangingConfig Ranging configuration to be used (same for all sensors)
 */
int RangeDemo(int UseSensorsMask, RangingConfig_e rangingConfig)
{
    int over=0;
    int status;
    char StrDisplay[5];
    char c;
    int i;
    int nSensorToUse;
    int SingleSensorNo=0;

    /* Setup all sensors in Single Shot mode */
    /* Which sensor to use ? */
    for(i=0, nSensorToUse=0; i<3; i++){
        if(( UseSensorsMask& (1<<i) ) && VL53L0XDevs[i].Present ){
            nSensorToUse++;
            if( nSensorToUse==1 )
                SingleSensorNo=i;
        }
    }
    if( nSensorToUse == 0 ){
        return -1;
    }


    /* Start ranging until blue button is pressed */
    if( nSensorToUse >1 ){
        /* Multiple devices */
        for( i=0; i<3; i++){
            if( ! VL53L0XDevs[i].Present  || (UseSensorsMask & (1<<i))==0 )
                continue;
            /* Call All-In-One blocking API function */
            status = VL53L0X_PerformSingleRangingMeasurement(&VL53L0XDevs[i],&RangingMeasurementData);
            if( status ){
                HandleError(ERR_DEMO_RANGE_MULTI);
            }
            /* Push data logging to UART */
            trace_printf("%d,%u,%d,%d,%d\n", VL53L0XDevs[i].Id, TimeStamp_Get(), RangingMeasurementData.RangeStatus, RangingMeasurementData.RangeMilliMeter, RangingMeasurementData.SignalRateRtnMegaCps);
            /* Store new ranging distance */
            Sensor_SetNewRange(&VL53L0XDevs[i],&RangingMeasurementData);
        }
    }
    else{
        /* only one sensor */
        /* Call All-In-One blocking API function */
        status = VL53L0X_PerformSingleRangingMeasurement(&VL53L0XDevs[SingleSensorNo],&RangingMeasurementData);
        if( status == 0 ){
            /* Push data logging to UART */
            trace_printf("%d,%u,%d,%d,%d\n", VL53L0XDevs[SingleSensorNo].Id, TimeStamp_Get(), RangingMeasurementData.RangeStatus, RangingMeasurementData.RangeMilliMeter, RangingMeasurementData.SignalRateRtnMegaCps);
            Sensor_SetNewRange(&VL53L0XDevs[SingleSensorNo],&RangingMeasurementData);
        }
        else{
            HandleError(ERR_DEMO_RANGE_ONE);
        }
        return 1;
    }
    return status;
}

static void InitSensors(void)
{
	    int i;
	    uint16_t Id= 0xEEAA;
	    int status;
	    int FinalAddress;

	    char PresentMsg[5]="    ";
	    /* Reset all */
	    nDevPresent = 0;
	    for (i = 0; i < 3; i++)
	        status = ResetId(i, 0);

	    HAL_Delay(2);
	    /* detect all sensors (even on-board)*/
	    for (i = 1; i >= 0; i--) {
	        VL53L0X_Dev_t *pDev;
	        pDev = &VL53L0XDevs[i];
	        pDev->I2cDevAddr = 0x52;
	        pDev->Present = 0;
	        status = ResetId( pDev->Id, 1); // XNUCLEO53L0A1_ResetId
	        HAL_Delay(2);
	        FinalAddress=0x52+(i+1)*5;

	        do {
	        	/* Set I2C standard mode (400 KHz) before doing the first register access */
	        	if (status == VL53L0X_ERROR_NONE)
	        		status = VL53L0X_WrByte(pDev, 0x88, 0x00);

	        	/* Try to read one register using default 0x52 address */
	            status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
	            if (status) {
	                debug_printf("#%d Read id fail\n", i);
	                break;
	            }
	            if (Id == 0xEEAA) {
					/* Sensor is found => Change its I2C address to final one */
	                status = VL53L0X_SetDeviceAddress(pDev,FinalAddress);
	                if (status != 0) {
	                    debug_printf("#i VL53L0X_SetDeviceAddress fail\n", i);
	                    break;
	                }
	                pDev->I2cDevAddr = FinalAddress;
	                /* Check all is OK with the new I2C address and initialize the sensor */
	                status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
	                if (status != 0) {
						debug_printf("#i VL53L0X_RdWord fail\n", i);
						break;
					}

	                status = VL53L0X_DataInit(pDev);
	                if( status == 0 ){
	                    pDev->Present = 1;
	                }
	                else{
	                    debug_printf("VL53L0X_DataInit %d fail\n", i);
	                    break;
	                }
	                trace_printf("VL53L0X %d Present and initiated to final 0x%x\n", pDev->Id, pDev->I2cDevAddr);
	                nDevPresent++;
	                nDevMask |= 1 << i;
	                pDev->Present = 1;
	            }
	            else {
	                debug_printf("#%d unknown ID %x\n", i, Id);
	                status = 1;
	            }

	        } while (0);
	        /* if fail r can't use for any reason then put the  device back to reset */
	        if (status) {
	        	ResetId(i, 0);
	        }
	    }
	    int nSensor = nDevPresent;
	    if( (nSensor <=0) ){
	        HandleError(ERR_DETECT);
	    }
  }

#if HAVE_ALARM_DEMO
struct AlrmMode_t {
    const int VL53L0X_Mode;
    const char *Name;
    uint32_t ThreshLow;
    uint32_t ThreshHigh;
};

struct AlrmMode_t AlarmModes[]={
        { .VL53L0X_Mode = VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW , .Name="Lo" , .ThreshLow=300<<16 ,  .ThreshHigh=0<<16  },
        { .VL53L0X_Mode = VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH, .Name= "hi", .ThreshLow=0<<16   ,  .ThreshHigh=300<<16},
        { .VL53L0X_Mode = VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT , .Name="out", .ThreshLow=300<<16 ,  .ThreshHigh=400<<16},
};

VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t StopCompleted=0;
    uint32_t LoopNb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
            if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }

    }

    return Status;
}


void AlarmDemo(void){
    VL53L0X_Dev_t *pDev;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
	uint8_t isApertureSpads;
	VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    int status;
    int Over=0;
    int Mode=0;
    char StrDisplay[5]="----";

    /* Only center device is used */
    pDev=&VL53L0XDevs[1];


    /* configure BSP/MCU center sensor interrupt */
    VL53L0A1_EXTI_IOConfigure(XNUCLEO53L0A1_DEV_CENTER, 0, 0);
    XNUCLEO53L0A1_SetIntrStateId(1, XNUCLEO53L0A1_DEV_CENTER);

    /* Initialize the device in continuous ranging mode */
	VL53L0X_StaticInit(pDev);
	VL53L0X_PerformRefCalibration(pDev, &VhvSettings, &PhaseCal);
	VL53L0X_PerformRefSpadManagement(pDev, &refSpadCount, &isApertureSpads);
	VL53L0X_SetInterMeasurementPeriodMilliSeconds(pDev, 250);
	VL53L0X_SetDeviceMode(pDev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

    do{
       /* set sensor interrupt mode */
       VL53L0X_StopMeasurement(pDev);           // it is safer to do this while sensor is stopped
       VL53L0X_SetInterruptThresholds(pDev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING ,  AlarmModes[Mode].ThreshLow ,  AlarmModes[Mode].ThreshHigh);
       status = VL53L0X_SetGpioConfig(pDev, 0, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, AlarmModes[Mode].VL53L0X_Mode, VL53L0X_INTERRUPTPOLARITY_HIGH);
       status = VL53L0X_ClearInterruptMask(pDev, -1); // clear interrupt pending if any

       /* Start continuous ranging */
       VL53L0X_StartMeasurement(pDev);
       IntrCounts[1]=0;

       /* Check for interrupt */
       do{
           __WFI();
           /* Interrupt received */
           if( IntrCounts[1] !=0 ){
        	   /* Reset interrupt counter */
               IntrCounts[1]=0;
               /* Get ranging data and display distance*/
               VL53L0X_GetRangingMeasurementData(pDev, &RangingMeasurementData);
               sprintf(StrDisplay, "%3dc",(int)RangingMeasurementData.RangeMilliMeter/10);
               /* Clear interrupt */
               status = VL53L0X_ClearInterruptMask(pDev, -1);
               /* keep display for at least 100ms otherwise user may never see it on display*/
               XNUCLEO53L0A1_SetDisplayString(StrDisplay);
               HAL_Delay(100);
           }
           else{
        	   /* No interrupt received => Display alarm mode */
               XNUCLEO53L0A1_SetDisplayString(AlarmModes[Mode].Name);
           }
           /* Check blue button */
           if( !BSP_GetPushButton() ){
               break;
           }
       }while(1);
       /* Wait button to be released to decide if it is a short or long press */
       status=PusbButton_WaitUnPress();
       /* Long press => stop this demo */
       if( status )
           Over =1;
       /* Short press => change alarm mode */
       Mode=(Mode+1)%ARRAY_SIZE(AlarmModes);
    }while( !Over );

    /* Stop continuous ranging */
    VL53L0X_StopMeasurement(pDev);

    /* Ensure device is ready for other commands */
    WaitStopCompleted(pDev);

    /* Disable configuration of BSP/MCU center sensor interrupt */
    XNUCLEO53L0A1_SetIntrStateId(0, XNUCLEO53L0A1_DEV_CENTER);
    VL53L0A1_EXTI_IOUnconfigure(XNUCLEO53L0A1_DEV_CENTER);
}
#endif

static int ResetId(int DevNo, GPIO_PinState state) {
	switch( DevNo )
	{
		case XNUCLEO53L0A1_DEV_LEFT :
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, state);
			break;

		case XNUCLEO53L0A1_DEV_CENTER :
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, state);
			break;
		default :
			break;
	}
	return HAL_OK;
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
  RangingConfig_e RangingConfig = LONG_RANGE;
  DemoMode_e DemoMode = RANGE_VALUE;
//  int UseSensorsMask = 1<<XNUCLEO53L0A1_DEV_CENTER;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

  uart_printf(WelcomeMsg);
  HAL_Delay(WelcomeTime);
//  (void)ResetId( 0, 1);
//  Check_I2c_Channel(&hi2c3);

  /* Set VL53L0X API trace level */
  VL53L0X_trace_config(NULL, TRACE_MODULE_NONE, TRACE_LEVEL_NONE, TRACE_FUNCTION_NONE); // No Trace
  //VL53L0X_trace_config(NULL,TRACE_MODULE_ALL, TRACE_LEVEL_ALL, TRACE_FUNCTION_ALL); // Full trace

  InitSensors();
  SetupSingleShot(RangingConfig);


//  HAL_TIM_Base_Start_IT(&htim10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  __WFI();



	  (void)RangeDemo(UseSensorsMask, LONG_RANGE);
	  HAL_Delay(1000);


	  /* Display demo mode */
//      XNUCLEO53L0A1_SetDisplayString(DemoModeTxt[DemoMode]);
//      HAL_Delay(ModeChangeDispTime);
//
//      /* Display Ranging config */
//	  XNUCLEO53L0A1_SetDisplayString(RangingConfigTxt[RangingConfig]);
//	  HAL_Delay(ModeChangeDispTime);
//
//	  /* Reset and Detect all sensors */
////      ResetAndDetectSensor(0);
//
//      /* Reset Timestamping */
//      TimeStamp_Reset();

#if HAVE_ALARM_DEMO
      XNUCLEO53L0A1_SetDisplayString(TxtAlarm);
      HAL_Delay(ModeChangeDispTime);
      ResetAndDetectSensor(0);
      AlarmDemo();
#else

//      /* Start Ranging demo */
//      ExitWithLongPress = RangeDemo(UseSensorsMask, RangingConfig);
//
//      /* Blue button has been pressed (long or short press) */
//      if(ExitWithLongPress){
//    	  /* Long press : change demo mode if multiple sensors present*/
//    	  if( nDevPresent >1 ){
//    		  /* If more than one sensor is present then toggle demo mode */
//    		  DemoMode = (DemoMode == RANGE_VALUE) ? BAR_GRAPH : RANGE_VALUE;
//    		  UseSensorsMask = (DemoMode == BAR_GRAPH) ? 0x7 : 1<<XNUCLEO53L0A1_DEV_CENTER;
//    	  }
//      } else {
//    	  /* Short press : change ranging config */
//    	  RangingConfig = (RangingConfig == LONG_RANGE) ? HIGH_SPEED : ((RangingConfig == HIGH_SPEED) ? HIGH_ACCURACY : LONG_RANGE);
//      }
#endif
    /* USER CODE END WHILE */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  htim10.Init.Period = 8399;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  HAL_GPIO_WritePin(LD2__Chip_enable_GPIO_Port, LD2__Chip_enable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SENSOR_1_GPIO_Port, SENSOR_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Blue_Push_Button_Pin SENSOR_2_Pin */
  GPIO_InitStruct.Pin = Blue_Push_Button_Pin|SENSOR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2__Chip_enable_Pin */
  GPIO_InitStruct.Pin = LD2__Chip_enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2__Chip_enable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENSOR_1_Pin */
  GPIO_InitStruct.Pin = SENSOR_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENSOR_1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim10) {
	  (void)RangeDemo(UseSensorsMask, LONG_RANGE); // hall delay 2 blocking doesnt work in callback
	  // obecnie można zrobić z flagą i wydajność zbadać przy pomocy analizatora.

//	  __HAL_TIM_ENABLE(&htim10);
  }
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
