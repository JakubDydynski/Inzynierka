/*
 * algorithm.h
 *
 *  Created on: 5 Nov 2023
 *      Author: Jakub
 */

#ifndef INC_ALGORITHM_H_
#define INC_ALGORITHM_H_
#include "stdint.h"

#define SENSOR_NUM  2

struct sensor_alg_cfg
{
	uint16_t range;
	uint16_t stop_distance;
	uint8_t max_step;
};

int calcStep(uint8_t sensor);
_Bool isInRange(uint8_t sensor);

#endif /* INC_ALGORITHM_H_ */
