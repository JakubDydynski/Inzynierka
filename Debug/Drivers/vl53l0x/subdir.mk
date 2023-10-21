################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/vl53l0x/vl53l0x_api.c \
../Drivers/vl53l0x/vl53l0x_api_calibration.c \
../Drivers/vl53l0x/vl53l0x_api_core.c \
../Drivers/vl53l0x/vl53l0x_api_ranging.c \
../Drivers/vl53l0x/vl53l0x_api_strings.c \
../Drivers/vl53l0x/vl53l0x_platform_log.c 

OBJS += \
./Drivers/vl53l0x/vl53l0x_api.o \
./Drivers/vl53l0x/vl53l0x_api_calibration.o \
./Drivers/vl53l0x/vl53l0x_api_core.o \
./Drivers/vl53l0x/vl53l0x_api_ranging.o \
./Drivers/vl53l0x/vl53l0x_api_strings.o \
./Drivers/vl53l0x/vl53l0x_platform_log.o 

C_DEPS += \
./Drivers/vl53l0x/vl53l0x_api.d \
./Drivers/vl53l0x/vl53l0x_api_calibration.d \
./Drivers/vl53l0x/vl53l0x_api_core.d \
./Drivers/vl53l0x/vl53l0x_api_ranging.d \
./Drivers/vl53l0x/vl53l0x_api_strings.d \
./Drivers/vl53l0x/vl53l0x_platform_log.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/vl53l0x/%.o Drivers/vl53l0x/%.su: ../Drivers/vl53l0x/%.c Drivers/vl53l0x/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DXNUCLEO53L0A1_TRACE=1 -DTRACE_UART=1 -DVL53L0A1_HAVE_UART=1 -DVL53L0X_LOG_ENABLE -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Dependencies -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/X-NUCLEO-53L0A1 -I../Drivers/BSP/vl53l0x -include../Drivers/BSP/X-NUCLEO-53L0A1/vl53l0x_platform.h -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-vl53l0x

clean-Drivers-2f-vl53l0x:
	-$(RM) ./Drivers/vl53l0x/vl53l0x_api.d ./Drivers/vl53l0x/vl53l0x_api.o ./Drivers/vl53l0x/vl53l0x_api.su ./Drivers/vl53l0x/vl53l0x_api_calibration.d ./Drivers/vl53l0x/vl53l0x_api_calibration.o ./Drivers/vl53l0x/vl53l0x_api_calibration.su ./Drivers/vl53l0x/vl53l0x_api_core.d ./Drivers/vl53l0x/vl53l0x_api_core.o ./Drivers/vl53l0x/vl53l0x_api_core.su ./Drivers/vl53l0x/vl53l0x_api_ranging.d ./Drivers/vl53l0x/vl53l0x_api_ranging.o ./Drivers/vl53l0x/vl53l0x_api_ranging.su ./Drivers/vl53l0x/vl53l0x_api_strings.d ./Drivers/vl53l0x/vl53l0x_api_strings.o ./Drivers/vl53l0x/vl53l0x_api_strings.su ./Drivers/vl53l0x/vl53l0x_platform_log.d ./Drivers/vl53l0x/vl53l0x_platform_log.o ./Drivers/vl53l0x/vl53l0x_platform_log.su

.PHONY: clean-Drivers-2f-vl53l0x

