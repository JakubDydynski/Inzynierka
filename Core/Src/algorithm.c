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
// what is equation of velocity for the model of train v = pwm*t?
#define V_MAX 10
#define ACCELERATION 2
#define TRAIN_LENGHT 200 // approx 200-210mm
#define LINE_LENGTH 600//10*180 // 10 torów każdy ok 180mm
#define TRAIN_MID_POINT ((TRAIN_LENGHT) / 2)
#define MIN_OFFSET ((TRAIN_MID_POINT) + 50)
//#define LENGTH_WITH_OFFSET ((LINE_LENGTH) + (MIN_OFFSET)*2)

//#define V_DELAY(distance) (sqrt(V_MAX) - sqrt(2 * ((distance - ((LENGTH_WITH_OFFSET) / 2))) * (ACCELERATION)))
//#define V_ACCELERATION(distance) (sqrt(2*(distance - (MIN_OFFSET))*(ACCELERATION)))
#define STEPS_NUM 8
#define shorten_step 3
#define DIST_PER_STEP ((LINE_LENGTH))/((STEPS_NUM)*shorten_step) //
volatile int dist_per_step = DIST_PER_STEP;
#define MAX_STEPS ((LINE_LENGTH)/(DIST_PER_STEP))
volatile int max_step = MAX_STEPS;

#define shorten_deacc 3
#define CALC_STEPS_UP(distance) (distance/(DIST_PER_STEP))
#define CALC_STEPS_DOWN(distance) ((MAX_STEPS) - (((LINE_LENGTH/2) - distance)/(DIST_PER_STEP/shorten_deacc)))

extern VL53L0X_RangingMeasurementData_t RangingMeasurementData[2];

enum sensor {ZERO, ONE};
int pwm = 0;
int calc_pos;
enum sensor sensor_meas = ONE;
static int calculate_position(void)
{
    calc_pos = RangingMeasurementData[0].RangeMilliMeter < RangingMeasurementData[1].RangeMilliMeter ?
    				RangingMeasurementData[0].RangeMilliMeter:
					RangingMeasurementData[1].RangeMilliMeter;
    return RangingMeasurementData[0].RangeMilliMeter < RangingMeasurementData[1].RangeMilliMeter ? ZERO : ONE;
}

int getInitialDir()
{
	enum dir {fwd, rev};
	enum dir dir;
	int position = calculate_position();

	if (position < LINE_LENGTH/2)
	{
		dir = fwd;
	}
	else
	{
		dir = rev;
	}
	return dir;
}


int calculatePWM(int *isStop)
{

    int position;
    //calc_pos = calculate_position();
    sensor_meas = calculate_position();
    if (sensor_meas == ONE) // acceleration phase
    {
        pwm = CALC_STEPS_UP(calc_pos);
    }
    else // deacceleration phase
    {
        pwm = CALC_STEPS_DOWN(calc_pos);
        if (pwm < 1)
        {
        	*isStop = 1;
        }
        else
        {
        	*isStop = 0;
        }
    }
    if (pwm<0)
    	return 0;
    return pwm;
}
