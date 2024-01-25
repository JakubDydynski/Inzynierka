/*
 * sensor.h
 *
 *  Created on: Oct 21, 2023
 *      Author: Jakub
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "vl53l0x_api.h"
#include "stm32f4xx_hal.h"
enum XNUCLEO53L0A1_dev_e{
    XNUCLEO53L0A1_DEV_LEFT =  0,    //!< left satellite device P21 header : 'l'
    XNUCLEO53L0A1_DEV_CENTER  =  1, //!< center (built-in) vl053 device : 'c"
    XNUCLEO53L0A1_DEV_RIGHT=  2     //!< Right satellite device P22 header : 'r'
};
typedef enum
{
	LONG_RANGE = 0, /*!< Long range mode */
	HIGH_SPEED = 1, /*!< High speed mode */
	HIGH_ACCURACY = 2, /*!< High accuracy mode */
} RangingConfig_e;
int SetShutdownPin(int DevNo, GPIO_PinState state);
int InitSensorsL1(uint16_t dev);
int StartSensorsL1(uint16_t dev);
void InitSensorsL0(I2C_HandleTypeDef *hi2c,  RangingConfig_e rangingConfig);
void OwnDemo(int UseSensorsMask);
void HandleError(int err);
void Sensor_SetNewRange(VL53L0X_Dev_t *pDev,
		VL53L0X_RangingMeasurementData_t *pRange);

#endif /* INC_SENSOR_H_ */
