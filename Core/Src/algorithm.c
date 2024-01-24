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
struct sensor_alg_cfg s_cfg[SENSOR_NUM] = {
	{380, 50, 20},
	{320, 70, 20}
};

_Bool isInRange(uint8_t sensor)
{
  _Bool ret = 1;
  ret &= RangingMeasurementData[sensor].RangeMilliMeter < s_cfg[sensor].range; // check in range
  ret &= RangingMeasurementData[sensor].SignalRateRtnMegaCps > 88000; // check signal rate is sufficient
  return ret;
}

int calcStep(uint8_t sensor)
{
  uint16_t distance = RangingMeasurementData[sensor].RangeMilliMeter;
  uint16_t step = (s_cfg[sensor].max_step - (s_cfg[sensor].range - (distance -
  s_cfg[sensor].stop_distance))/(s_cfg[sensor].range/s_cfg[sensor].max_step));
  return step;
}
