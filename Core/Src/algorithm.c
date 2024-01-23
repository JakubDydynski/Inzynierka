/*
 * algorithm.c
 *
 *  Created on: 5 Nov 2023
 *      Author: Jakub
 */
#include "algorithm.h"
#include "vl53l0x_api.h"
#include "stdint.h"
extern VL53L0X_RangingMeasurementData_t RangingMeasurementData[2];

//40-36 10
//36 9
//32 8
//28 7
//24 6
//20 5
//16 4
//12 3
//8 2
//4 1
//3 0
// we miss 1 dspeed bcs of stop distance
// y = b - a(x-coś)
#define MAX_STEP 20
#define STOP_DISTANCE 55 
#define SENSOR_RANGE 400 
#define SENSOR (sensor-1)
#define CALC_STEP(distance)	(MAX_STEP - (s_cfg[SENSOR].range - (distance - s_cfg[SENSOR].stop_distance))/(s_cfg[SENSOR].range/MAX_STEP))
#define GET_DISTANCE(sensor) RangingMeasurementData[SENSOR].RangeMilliMeter

struct sensor_alg_cfg
{
	uint16_t range;
	uint16_t stop_distance;
};

const struct sensor_alg_cfg s_cfg[] = {
		{380, 50},
		{320, 70}
};

_Bool isInRange(uint8_t sensor)
{
	return GET_DISTANCE(sensor) < s_cfg[SENSOR].range ? 1 : 0;
}
uint16_t calc_pos = 0;
uint16_t step;
int calcStep(uint8_t sensor)
{
	calc_pos = GET_DISTANCE(sensor);
	step = CALC_STEP(calc_pos);
    return step;
}
