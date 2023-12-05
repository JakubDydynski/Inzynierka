################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/algorithm.c \
../Core/Src/auxilary.c \
../Core/Src/flash.c \
../Core/Src/main.c \
../Core/Src/sensor.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/algorithm.o \
./Core/Src/auxilary.o \
./Core/Src/flash.o \
./Core/Src/main.o \
./Core/Src/sensor.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/algorithm.d \
./Core/Src/auxilary.d \
./Core/Src/flash.d \
./Core/Src/main.d \
./Core/Src/sensor.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DXNUCLEO53L0A1_TRACE=1 -DTRACE_UART=1 -DVL53L0A1_HAVE_UART=1 -DVL53L0X_LOG_ENABLE -DUSE_HAL_DRIVER -DSTM32F401xE -DDCON1a -c -I../Core/Inc -I../Dependencies -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/X-NUCLEO-53L0A1 -I../Drivers/BSP/vl53l0x -I"C:/Users/Jakub/Documents/vsc/STM32F401Nucleo-VL6180X/DCCCctrl/Inc" -I"C:/Users/Jakub/Documents/vsc/STM32F401Nucleo-VL6180X/Blocks/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/algorithm.d ./Core/Src/algorithm.o ./Core/Src/algorithm.su ./Core/Src/auxilary.d ./Core/Src/auxilary.o ./Core/Src/auxilary.su ./Core/Src/flash.d ./Core/Src/flash.o ./Core/Src/flash.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/sensor.d ./Core/Src/sensor.o ./Core/Src/sensor.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

