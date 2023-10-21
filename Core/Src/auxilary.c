/*
 * auxilary.c
 *
 *  Created on: Jul 27, 2023
 *      Author: Jakub
 */
#include "auxilary.h"
#include "vl53l0x_platform_log.h"
//	GPIOA->BSRR ^= (1 << 5);
//	for(volatile int i = 0; i < 15600; i++) {}
//	GPIOA->BSRR ^= (1 << (5+16));
//
//

void Check_I2c_Channel(I2C_HandleTypeDef* channel)
{
	    HAL_StatusTypeDef result;
	    uint8_t r = 0 , nr = 0;
		uint8_t i;
		for (i=1; i<128; i++)
		{
		  /*
		   * the HAL wants a left aligned i2c address
		   * &hi2c1 is the handle
		   * (uint16_t)(i<<1) is the i2c address left aligned
		   * retries 2
		   * timeout 2
		   */
		  result = HAL_I2C_IsDeviceReady(channel, (uint16_t)(i<<1), 2, 2);
		  if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
		  {
			  nr++;
			  uart_printf("."); // No ACK received at that address
		  }
		  if (result == HAL_OK)
		  {
			  r++;
			  uart_printf("0x%X", i); // Received an ACK at that address
		  }
		}
}

