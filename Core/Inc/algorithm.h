/*
 * algorithm.h
 *
 *  Created on: 5 Nov 2023
 *      Author: Jakub
 */

#ifndef INC_ALGORITHM_H_
#define INC_ALGORITHM_H_
#include "stdint.h"
int calcStep(uint8_t sensor);
_Bool getInitialDir();
_Bool isInRange(uint8_t sensor);

#endif /* INC_ALGORITHM_H_ */
