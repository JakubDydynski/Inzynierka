################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Blocks/Src/cmdedit.c \
../Blocks/Src/cmdparse.c \
../Blocks/Src/cmdproc.c 

OBJS += \
./Blocks/Src/cmdedit.o \
./Blocks/Src/cmdparse.o \
./Blocks/Src/cmdproc.o 

C_DEPS += \
./Blocks/Src/cmdedit.d \
./Blocks/Src/cmdparse.d \
./Blocks/Src/cmdproc.d 


# Each subdirectory must supply rules for building sources it contributes
Blocks/Src/%.o Blocks/Src/%.su: ../Blocks/Src/%.c Blocks/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DXNUCLEO53L0A1_TRACE=1 -DTRACE_UART=1 -DVL53L0A1_HAVE_UART=1 -DVL53L0X_LOG_ENABLE -DUSE_HAL_DRIVER -DSTM32F401xE -DDCON1a -c -I../Core/Inc -I../Dependencies -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/X-NUCLEO-53L0A1 -I../Drivers/BSP/vl53l0x -I"C:/Users/Jakub/Documents/vsc/STM32F401Nucleo-VL6180X/DCCCctrl/Inc" -I"C:/Users/Jakub/Documents/vsc/STM32F401Nucleo-VL6180X/Blocks/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Blocks-2f-Src

clean-Blocks-2f-Src:
	-$(RM) ./Blocks/Src/cmdedit.d ./Blocks/Src/cmdedit.o ./Blocks/Src/cmdedit.su ./Blocks/Src/cmdparse.d ./Blocks/Src/cmdparse.o ./Blocks/Src/cmdparse.su ./Blocks/Src/cmdproc.d ./Blocks/Src/cmdproc.o ./Blocks/Src/cmdproc.su

.PHONY: clean-Blocks-2f-Src

