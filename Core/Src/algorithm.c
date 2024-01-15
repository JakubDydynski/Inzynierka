/*
 * algorithm.c
 *
 *  Created on: 5 Nov 2023
 *      Author: Jakub
 */
#include "algorithm.h"
#include "X-NUCLEO-53L0A1.h"
#include "vl53l0x_api.h"
#include "math.h"
#include "stdint.h"
extern VL53L0X_RangingMeasurementData_t RangingMeasurementData[2];

//#define STEPS_NUM 8
//#define shorten_step 3
//#define DIST_PER_STEP ((LINE_LENGTH))/((STEPS_NUM)*shorten_step) //
//volatile int dist_per_step = DIST_PER_STEP;
//#define MAX_STEPS ((LINE_LENGTH)/(DIST_PER_STEP))
//volatile int max_step = MAX_STEPS;
//
//#define shorten_deacc 3
//#define CALC_STEPS_UP(distance) (distance/(DIST_PER_STEP))
//#define CALC_STEPS_DOWN(distance) ((MAX_STEPS) - (((LINE_LENGTH/2) - distance)/(DIST_PER_STEP/shorten_deacc)))


#define TRAIN_LENGHT 200 // approx 200-210mm
#define LINE_LENGTH 600//10*180 // 10 torów każdy ok 180mm
#define MAX_STEP 16
#define CALC_STEP(distance)	((MAX_STEP - (SENSOR_RANGE - (distance - STOP_DISTANCE))/(SENSOR_RANGE/MAX_STEP))+1) // na 40cm mamy 10stepów i co 4 cm zmienjaszamy o 1 step
#define GET_DISTANCE(sensor) RangingMeasurementData[sensor-1].RangeMilliMeter
#define SENSOR_RANGE 400 //400mm
#define STOP_DISTANCE 55 // 55mm
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
int step = 0;
uint16_t calc_pos;

_Bool isInRange(uint8_t sensor)
{
	return GET_DISTANCE(sensor) < SENSOR_RANGE ? 1 : 0;
}

int calcStep(uint8_t sensor)
{
	calc_pos = GET_DISTANCE(sensor);

	step = CALC_STEP(calc_pos);
	if (calc_pos < STOP_DISTANCE)
		step = 0;

    return step;
}
