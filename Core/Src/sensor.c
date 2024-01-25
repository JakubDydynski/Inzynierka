/*
 * sensor.c
 *
 *  Created on: Oct 21, 2023
 *      Author: Jakub
 */
#include "sensor.h"
#include "stm32f4xx_hal.h"
#include "vl53l0x_api.h"
#include "VL53L1X_API.h"
#include "VL53l1X_calibration.h"
/**
 * @defgroup ErrCode Errors code shown on display
 * @{
 */

#define ERR_DETECT             -1
#define debug_printf    trace_printf

/** bit is index in VL53L0XDevs that is not necessary the dev id of the BSP */
int nDevMask;

/** How many device detect set by @a DetectSensors()*/
int nDevPresent = 0;

/** leaky factor for filtered range
 *
 * r(n) = averaged_r(n-1)*leaky +r(n)(1-leaky)
 *
 * */
int LeakyFactorFix8 = (int) (0.6 * 256);


/**
 * Handle Error
 *
 * Set err on display and loop forever
 * @param err Error case code
 */
void HandleError(int err)
{
//	char msg[16];
//	sprintf(msg, "Er%d", err);
//	XNUCLEO53L0A1_SetDisplayString(msg);
	while (1)
	{
	};
}

VL53L0X_Dev_t VL53L0XDevs[2] =
{
    { .Id = XNUCLEO53L0A1_DEV_LEFT, .DevLetter = 'l', .I2cDevAddr = 0x52 },
    { .Id = XNUCLEO53L0A1_DEV_CENTER, .DevLetter = 'c', .I2cDevAddr = 0x52 }, 
};

int SetShutdownPin(int DevNo, GPIO_PinState state)
{
	switch (DevNo)
	{
	case XNUCLEO53L0A1_DEV_LEFT:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, state);
		break;

	case XNUCLEO53L0A1_DEV_CENTER:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, state);
		break;
	default:
		break;
	}
	return HAL_OK;
}

/**
 *  Setup all detected sensors for single shot mode and setup ranging configuration
 */
static void SetupSingleShot(RangingConfig_e rangingConfig)
{
	int i;
	int status;
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	FixPoint1616_t signalLimit = (FixPoint1616_t) (0.25 * 65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t) (18 * 65536);
	uint32_t timingBudget = 33000;
	uint8_t preRangeVcselPeriod = 14;
	uint8_t finalRangeVcselPeriod = 10;

	for (i = 0; i < 2; i++)
	{
		if (VL53L0XDevs[i].Present)
		{
			status = VL53L0X_StaticInit(&VL53L0XDevs[i]);
			if (status)
			{
				debug_printf("VL53L0X_StaticInit %d failed\n", i);
			}

			status = VL53L0X_PerformRefCalibration(&VL53L0XDevs[i],
					&VhvSettings, &PhaseCal);
			if (status)
			{
				debug_printf("VL53L0X_PerformRefCalibration failed\n");
			}

			status = VL53L0X_PerformRefSpadManagement(&VL53L0XDevs[i],
					&refSpadCount, &isApertureSpads);
			if (status)
			{
				debug_printf("VL53L0X_PerformRefSpadManagement failed\n");
			}

			status = VL53L0X_SetDeviceMode(&VL53L0XDevs[i],
			VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
			if (status)
			{
				debug_printf("VL53L0X_SetDeviceMode failed\n");
			}

			status = VL53L0X_SetLimitCheckEnable(&VL53L0XDevs[i],
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
			if (status)
			{
				debug_printf("VL53L0X_SetLimitCheckEnable failed\n");
			}

			status = VL53L0X_SetLimitCheckEnable(&VL53L0XDevs[i],
			VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit
			if (status)
			{
				debug_printf("VL53L0X_SetLimitCheckEnable failed\n");
			}
			/* Ranging configuration */
			switch (rangingConfig)
			{
			case LONG_RANGE:
				signalLimit = (FixPoint1616_t) (0.1 * 65536);
				sigmaLimit = (FixPoint1616_t) (60 * 65536);
				timingBudget = 33000;
				preRangeVcselPeriod = 18;
				finalRangeVcselPeriod = 14;
				break;
			case HIGH_ACCURACY:
				signalLimit = (FixPoint1616_t) (0.25 * 65536);
				sigmaLimit = (FixPoint1616_t) (18 * 65536);
				timingBudget = 200000;
				preRangeVcselPeriod = 14;
				finalRangeVcselPeriod = 10;
				break;
			case HIGH_SPEED:
				signalLimit = (FixPoint1616_t) (0.25 * 65536);
				sigmaLimit = (FixPoint1616_t) (32 * 65536);
				timingBudget = 20000;
				preRangeVcselPeriod = 14;
				finalRangeVcselPeriod = 10;
				break;
			default:
				debug_printf("Not Supported");
			}

			status = VL53L0X_SetLimitCheckValue(&VL53L0XDevs[i],
			VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
			if (status)
			{
				debug_printf("VL53L0X_SetLimitCheckValue failed\n");
			}

			status = VL53L0X_SetLimitCheckValue(&VL53L0XDevs[i],
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
			if (status)
			{
				debug_printf("VL53L0X_SetLimitCheckValue failed\n");
			}

			status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(
					&VL53L0XDevs[i], timingBudget);
			if (status)
			{
				debug_printf(
						"VL53L0X_SetMeasurementTimingBudgetMicroSeconds failed\n");
			}

			status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDevs[i],
			VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
			if (status)
			{
				debug_printf("VL53L0X_SetVcselPulsePeriod failed\n");
			}

			status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDevs[i],
			VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);
			if (status)
			{
				debug_printf("VL53L0X_SetVcselPulsePeriod failed\n");
			}

			status = VL53L0X_PerformRefCalibration(&VL53L0XDevs[i],
					&VhvSettings, &PhaseCal);
			if (status)
			{
				debug_printf("VL53L0X_PerformRefCalibration failed\n");
			}

			VL53L0XDevs[i].LeakyFirst = 1;
		}
	}
}

/* Store new ranging data into the device structure, apply leaky integrator if needed */
void Sensor_SetNewRange(VL53L0X_Dev_t *pDev,
		VL53L0X_RangingMeasurementData_t *pRange)
{
	if (pRange->RangeStatus == 0)
	{
		if (pDev->LeakyFirst)
		{
			pDev->LeakyFirst = 0;
			pDev->LeakyRange = pRange->RangeMilliMeter;
		} else
		{
			pDev->LeakyRange = (pDev->LeakyRange * LeakyFactorFix8
					+ (256 - LeakyFactorFix8) * pRange->RangeMilliMeter) >> 8;
		}
	} else
	{
		pDev->LeakyFirst = 1;
	}
}
int InitSensorsL1(uint16_t dev)
{
	int status;
	uint8_t sensorState=0;
	int i;
	for (i = 0; i < 2; i++)
			(void)SetShutdownPin(i, 0);
	for (i = 0; i < 2; i++)
	{

		(void)SetShutdownPin(i, 0);
		HAL_Delay(2);
		(void)SetShutdownPin(i, 1);
		HAL_Delay(2);
		status = VL53L1X_SetI2CAddress(dev, dev + (i+1)*2);

	/* Those basic I2C read functions can be used to check your own I2C functions */
//		status = VL53L1_RdByte(dev + (i+1)*2, 0x010F, &byteData);
//		printf("VL53L1X Model_ID: %X\n", byteData);
//		status = VL53L1_RdByte(dev + (i+1)*2, 0x0110, &byteData);
//		printf("VL53L1X Module_Type: %X\n", byteData);
//		status = VL53L1_RdWord(dev + (i+1)*2, 0x010F, &wordData);
//		printf("VL53L1X: %X\n", wordData);
		while(sensorState==0){
			status = VL53L1X_BootState(dev + (i+1)*2, &sensorState);
		HAL_Delay(2);
		}

		/* This function must to be called to initialize the sensor with the default setting  */
		status = VL53L1X_SensorInit(dev + (i+1)*2);
		/* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
		status = VL53L1X_SetDistanceMode(dev + (i+1)*2, 2); /* 1=short, 2=long */
		/*timinig budget doesnt play big role */
		status = VL53L1X_SetTimingBudgetInMs(dev + (i+1)*2, 100); /* in ms possible values [20, 50, 100, 200, 500] */
		status = VL53L1X_SetInterMeasurementInMs(dev + (i+1)*2, 100); /* in ms, IM must be > = TB */
		  status = VL53L1X_SetOffset(dev+4,20); /* offset compensation in mm */
		status = VL53L1X_SetROI(dev + (i+1)*2, 4, 4); /* minimum ROI 4,4 */
		//	status = VL53L1X_CalibrateOffset(dev, 140, &offset); /* may take few second to perform the offset cal*/
		//	status = VL53L1X_CalibrateXtalk(dev, 1000, &xtalk); /* may take few second to perform the xtalk cal */
	}

//	status = VL53L1X_GetXtalk(dev+4, &xtalk); /* may take few second to perform the xtalk cal */
//	status = VL53L1X_CalibrateXtalk(dev+4, 800, &xtalk); /* may take few second to perform the xtalk cal */
	/*xtalk is about 12208/12085 in above measurement*/
//	status = VL53L1X_GetXtalk(dev+4, &xtalk); /* may take few second to perform the xtalk cal */
//	status = VL53L1X_SetXtalk(dev+4, 12100); /* may take few second to perform the xtalk cal */
	return status;
}
int StartSensorsL1(uint16_t dev)
{
	int i;
	int status;
	for (i = 0; i < 2; i++)
	{
		status = VL53L1X_StartRanging(dev + (i+1)*2);   /* This function has to be called to enable the ranging */

	}
	return status;
}
void InitSensorsL0(I2C_HandleTypeDef *hi2c, RangingConfig_e rangingConfig)
{
	int i;
	uint16_t Id = 0xEEAA;
	int status;
	int FinalAddress;
	nDevPresent = 0;

    for(int i = 0; i < sizeof(VL53L0XDevs)/sizeof(VL53L0XDevs[0]); i++)
    {
        VL53L0XDevs[i].I2cHandle = hi2c;
    }

	/* Reset all */
	for (i = 0; i < 3; i++)
		status = SetShutdownPin(i, 0);
	HAL_Delay(2);

	/* detect all sensors */
	for (i = 0; i < 2; i++)
	{
		VL53L0X_Dev_t *pDev;
		pDev = &VL53L0XDevs[i];
		pDev->I2cDevAddr = 0x52;
		pDev->Present = 0;
		status = SetShutdownPin(pDev->Id, 1);
		HAL_Delay(2);
		FinalAddress = 0x52 + (i + 1) * 5;

		do
		{
			/* Set I2C standard mode (400 KHz) before doing the first register access */
			if (status == VL53L0X_ERROR_NONE)
				status = VL53L0X_WrByte(pDev, 0x88, 0x00);

			/* Try to read one register using default 0x52 address */
			status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID,
					&Id);
			if (status)
			{
				debug_printf("#%d Read id fail\n", i);
				break;
			}
			if (Id == 0xEEAA)
			{
				/* Sensor is found => Change its I2C address to final one */
				status = VL53L0X_SetDeviceAddress(pDev, FinalAddress);
				if (status != 0)
				{
					debug_printf("#i VL53L0X_SetDeviceAddress fail\n", i);
					break;
				}
				pDev->I2cDevAddr = FinalAddress;
				/* Check all is OK with the new I2C address and initialize the sensor */
				status = VL53L0X_RdWord(pDev,
				VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
				if (status != 0)
				{
					debug_printf("#i VL53L0X_RdWord fail\n", i);
					break;
				}

				status = VL53L0X_DataInit(pDev);
				if (status == 0)
				{
					pDev->Present = 1;
				} else
				{
					debug_printf("VL53L0X_DataInit %d fail\n", i);
					break;
				}
				trace_printf("VL53L0X %d Present and initiated to final 0x%x\n",
						pDev->Id, pDev->I2cDevAddr);
				nDevPresent++;
				nDevMask |= 1 << i;
				pDev->Present = 1;
			} 
            else
			{
				debug_printf("#%d unknown ID %x\n", i, Id);
				status = 1;
			}

		} while (0);
		/* if fail r can't use for any reason then put the  device back to reset */
		if (status)
		{
			SetShutdownPin(i, 0);
		}
	}
	if ((nDevPresent <= 0))
	{
		HandleError(ERR_DETECT);
		while(1);
	};

    SetupSingleShot(rangingConfig); // calibration

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
