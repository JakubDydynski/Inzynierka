################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/X-NUCLEO-53L0A1/X-NUCLEO-53L0A1.c \
../Drivers/BSP/X-NUCLEO-53L0A1/uart_trace.c \
../Drivers/BSP/X-NUCLEO-53L0A1/vl53l0a1-l053msp.c \
../Drivers/BSP/X-NUCLEO-53L0A1/vl53l0a1-x4msp.c \
../Drivers/BSP/X-NUCLEO-53L0A1/vl53l0x_platform.c 

OBJS += \
./Drivers/BSP/X-NUCLEO-53L0A1/X-NUCLEO-53L0A1.o \
./Drivers/BSP/X-NUCLEO-53L0A1/uart_trace.o \
./Drivers/BSP/X-NUCLEO-53L0A1/vl53l0a1-l053msp.o \
./Drivers/BSP/X-NUCLEO-53L0A1/vl53l0a1-x4msp.o \
./Drivers/BSP/X-NUCLEO-53L0A1/vl53l0x_platform.o 

C_DEPS += \
./Drivers/BSP/X-NUCLEO-53L0A1/X-NUCLEO-53L0A1.d \
./Drivers/BSP/X-NUCLEO-53L0A1/uart_trace.d \
./Drivers/BSP/X-NUCLEO-53L0A1/vl53l0a1-l053msp.d \
./Drivers/BSP/X-NUCLEO-53L0A1/vl53l0a1-x4msp.d \
./Drivers/BSP/X-NUCLEO-53L0A1/vl53l0x_platform.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/X-NUCLEO-53L0A1/%.o Drivers/BSP/X-NUCLEO-53L0A1/%.su: ../Drivers/BSP/X-NUCLEO-53L0A1/%.c Drivers/BSP/X-NUCLEO-53L0A1/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DXNUCLEO53L0A1_TRACE=1 -DTRACE_UART=1 -DVL53L0A1_HAVE_UART=1 -DVL53L0X_LOG_ENABLE -DUSE_HAL_DRIVER -DSTM32F401xE -DDCON1a -c -I../Core/Inc -I../Dependencies -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/X-NUCLEO-53L0A1 -I../Drivers/BSP/vl53l0x -I"C:/Users/Jakub/Documents/vsc/STM32F401Nucleo-VL6180X/DCCCctrl/Inc" -I"C:/Users/Jakub/Documents/vsc/STM32F401Nucleo-VL6180X/Blocks/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-X-2d-NUCLEO-2d-53L0A1

clean-Drivers-2f-BSP-2f-X-2d-NUCLEO-2d-53L0A1:
	-$(RM) ./Drivers/BSP/X-NUCLEO-53L0A1/X-NUCLEO-53L0A1.d ./Drivers/BSP/X-NUCLEO-53L0A1/X-NUCLEO-53L0A1.o ./Drivers/BSP/X-NUCLEO-53L0A1/X-NUCLEO-53L0A1.su ./Drivers/BSP/X-NUCLEO-53L0A1/uart_trace.d ./Drivers/BSP/X-NUCLEO-53L0A1/uart_trace.o ./Drivers/BSP/X-NUCLEO-53L0A1/uart_trace.su ./Drivers/BSP/X-NUCLEO-53L0A1/vl53l0a1-l053msp.d ./Drivers/BSP/X-NUCLEO-53L0A1/vl53l0a1-l053msp.o ./Drivers/BSP/X-NUCLEO-53L0A1/vl53l0a1-l053msp.su ./Drivers/BSP/X-NUCLEO-53L0A1/vl53l0a1-x4msp.d ./Drivers/BSP/X-NUCLEO-53L0A1/vl53l0a1-x4msp.o ./Drivers/BSP/X-NUCLEO-53L0A1/vl53l0a1-x4msp.su ./Drivers/BSP/X-NUCLEO-53L0A1/vl53l0x_platform.d ./Drivers/BSP/X-NUCLEO-53L0A1/vl53l0x_platform.o ./Drivers/BSP/X-NUCLEO-53L0A1/vl53l0x_platform.su

.PHONY: clean-Drivers-2f-BSP-2f-X-2d-NUCLEO-2d-53L0A1

