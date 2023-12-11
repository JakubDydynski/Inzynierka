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
#define TRAIN_LENGHT 20
#define TRAIN_MID_POINT ((TRAIN_LENGHT) / 2)
#define MIN_OFFSET ((TRAIN_MID_POINT) + 5)
#define DISTANCE_W_OFFSET (100 + (MIN_OFFSET)*2)

#define V_DELAY(distance) (sqrt(V_MAX) - sqrt(2 * ((distance - ((DISTANCE_W_OFFSET) / 2))) * (ACCELERATION)))
#define V_ACCELERATION(distance) (sqrt(2*(distance - (MIN_OFFSET))*(ACCELERATION)))

extern VL53L0X_RangingMeasurementData_t RangingMeasurementData[2];

static int calculate_position(void)
{
    int position = RangingMeasurementData[0].RangeMilliMeter < RangingMeasurementData[1].RangeMilliMeter ?
    				RangingMeasurementData[0].RangeMilliMeter + TRAIN_MID_POINT:
					RangingMeasurementData[1].RangeMilliMeter - TRAIN_MID_POINT;
    return position; 
}

int getInitialDir()
{
	enum dir {fwd, rev};
	enum dir dir;
	int position = calculate_position();

	if (position < DISTANCE_W_OFFSET/2)
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
    int pwm;
    int position;
    position = calculate_position();

    if (position < DISTANCE_W_OFFSET/2) // acceleration phase
    {
        pwm = V_ACCELERATION(position);
    }
    else // deacceleration phase
    {
        pwm = V_DELAY(position);
        if (pwm < 3)
        {
        	*isStop = 1;
        }
        else
        {
        	*isStop = 0;
        }
    }

    return pwm;
}
