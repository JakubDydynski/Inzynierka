/*
 * auxilary.c
 *
 *  Created on: Jul 27, 2023
 *      Author: Jakub
 */
#include "auxilary.h"
#include "vl53l0x_platform_log.h"
//	GPIOA->BSRR ^= (1 << 5);
//	for(volatile int i = 0; i < 15600; i++) {}
//	GPIOA->BSRR ^= (1 << (5+16));
//
//

void Check_I2c_Channel(I2C_HandleTypeDef* channel)
{
	    HAL_StatusTypeDef result;
	    uint8_t r = 0 , nr = 0;
		uint8_t i;
		for (i=1; i<128; i++)
		{
		  /*
		   * the HAL wants a left aligned i2c address
		   * &hi2c1 is the handle
		   * (uint16_t)(i<<1) is the i2c address left aligned
		   * retries 2
		   * timeout 2
		   */
		  result = HAL_I2C_IsDeviceReady(channel, (uint16_t)(i<<1), 2, 2);
		  if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
		  {
			  nr++;
			  uart_printf("."); // No ACK received at that address
		  }
		  if (result == HAL_OK)
		  {
			  r++;
			  uart_printf("0x%X", i); // Received an ACK at that address
		  }
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