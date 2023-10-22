/*
 * auxilary.h
 *
 *  Created on: Jul 27, 2023
 *      Author: Jakub
 */

#ifndef INC_AUXILARY_H_
#define INC_AUXILARY_H_

#include "main.h"
void Check_I2c_Channel(I2C_HandleTypeDef* channel);

#define TEST_PIN_LOW() \
do \
{\
 GPIOA->BSRR ^= (1 << (5+16));\
}while(0);

#define TEST_PIN_HIGH() \
do \
{\
 GPIOA->BSRR ^= (1 << 5); \
}while(0);
#endif /* INC_AUXILARY_H_ */
