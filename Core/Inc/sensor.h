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

typedef enum
{
	LONG_RANGE = 0, /*!< Long range mode */
	HIGH_SPEED = 1, /*!< High speed mode */
	HIGH_ACCURACY = 2, /*!< High accuracy mode */
} RangingConfig_e;

void InitSensors(I2C_HandleTypeDef *hi2c,  RangingConfig_e rangingConfig);
void HandleError(int err);
void Sensor_SetNewRange(VL53L0X_Dev_t *pDev,
		VL53L0X_RangingMeasurementData_t *pRange);

VL53L0X_Dev_t VL53L0XDevs[2];

#endif /* INC_SENSOR_H_ */
