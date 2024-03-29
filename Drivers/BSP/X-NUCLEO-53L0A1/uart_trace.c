#include "stm32f4xx_hal.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "auxilary.h"
#ifndef TRACE_UART
/**
 * Force TRACE_UART to 0 by default (if not defined in the project)
 */
#define TRACE_UART  0
#endif

#if TRACE_UART
extern UART_HandleTypeDef huart2;

static volatile int InUsed=0;
static char uart_buffer[256];
static uint32_t UartErrCnt=0;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    // TODO check if any more to send and do it
    InUsed=0;
    TEST_PIN_HIGH();
}

int uart_vprintf(const char *msg, va_list ap){
    int n;
    int status;
    while( InUsed ){
           //
        __WFI();
    }
    InUsed|=1;
    n=vsnprintf(uart_buffer, sizeof(uart_buffer),  msg, ap);
    status = HAL_UART_Transmit_DMA(&huart2, (uint8_t*)uart_buffer, n );
    if( status ){
        UartErrCnt++;
        InUsed=0;
    }
    return n;
}

int uart_printf(const char *msg, ...){
	va_list ap;
    int n;
    while( InUsed ){
        //
        __WFI();
    }
    va_start(ap, msg);
    n=uart_vprintf(msg, ap);
    va_end(ap);
    return n;
}

void uart_printf_light(const char *msg, uint16_t size)
{
    while( InUsed ){
        //
        __WFI();
    }
    while( InUsed ){
           //
        __WFI();
    }
    InUsed|=1;

    // Copy the contents from sourceBuffer to destinationBuffer
    memcpy(uart_buffer, msg, size);

    // Ensure the destinationBuffer is null-terminated (if it's larger than sourceBuffer)
    uart_buffer[size - 1] = '\0';
    int status = HAL_UART_Transmit_DMA(&huart2, (uint8_t*)uart_buffer, size );
    if( status ){
        UartErrCnt++;
        InUsed=0;
    }
}
#else
#	define uart_vprintf(...) (void)0
#	define uart_printf(...)	(void)0
#endif
