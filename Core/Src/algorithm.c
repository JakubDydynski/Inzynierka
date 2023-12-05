/*
 * algorithm.c
 *
 *  Created on: 5 Nov 2023
 *      Author: Jakub
 */


#include "algorithm.h"
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

static int calculate_position(int sensor1Distance, int sensor2Distance)
{
    int position = sensor1Distance + sensor2Distance - TRAIN_MID_POINT;
    return position; 
}

int calculatePWM(int sensor1Distance, int sensor2Distance) 
{
    int pwm;
    int position;
    position = calculate_position(sensor1Distance, sensor2Distance);

    if (position < DISTANCE_W_OFFSET/2) // acceleration phase
    {
        pwm = V_ACCELERATION(position);
    }
    else // deacceleration phase
    {
        pwm = V_DELAY(position);
    }

    return pwm;
}