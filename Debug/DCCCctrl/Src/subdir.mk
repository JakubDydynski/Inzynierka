################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DCCCctrl/Src/dcc_auto.c \
../DCCCctrl/Src/dcc_tx.c \
../DCCCctrl/Src/dcc_tx_hw.c \
../DCCCctrl/Src/dccctrl_cmdi.c \
../DCCCctrl/Src/dccctrl_config.c \
../DCCCctrl/Src/dccctrl_timing.c \
../DCCCctrl/Src/sx_tx.c 

OBJS += \
./DCCCctrl/Src/dcc_auto.o \
./DCCCctrl/Src/dcc_tx.o \
./DCCCctrl/Src/dcc_tx_hw.o \
./DCCCctrl/Src/dccctrl_cmdi.o \
./DCCCctrl/Src/dccctrl_config.o \
./DCCCctrl/Src/dccctrl_timing.o \
./DCCCctrl/Src/sx_tx.o 

C_DEPS += \
./DCCCctrl/Src/dcc_auto.d \
./DCCCctrl/Src/dcc_tx.d \
./DCCCctrl/Src/dcc_tx_hw.d \
./DCCCctrl/Src/dccctrl_cmdi.d \
./DCCCctrl/Src/dccctrl_config.d \
./DCCCctrl/Src/dccctrl_timing.d \
./DCCCctrl/Src/sx_tx.d 


# Each subdirectory must supply rules for building sources it contributes
DCCCctrl/Src/%.o DCCCctrl/Src/%.su: ../DCCCctrl/Src/%.c DCCCctrl/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DXNUCLEO53L0A1_TRACE=1 -DTRACE_UART=1 -DVL53L0A1_HAVE_UART=1 -DVL53L0X_LOG_ENABLE -DUSE_HAL_DRIVER -DSTM32F401xE -DDCON1a -c -I../Core/Inc -I../Dependencies -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/X-NUCLEO-53L0A1 -I../Drivers/BSP/vl53l0x -I"C:/Users/Jakub/Documents/vsc/STM32F401Nucleo-VL6180X/DCCCctrl/Inc" -I"C:/Users/Jakub/Documents/vsc/STM32F401Nucleo-VL6180X/Blocks/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DCCCctrl-2f-Src

clean-DCCCctrl-2f-Src:
	-$(RM) ./DCCCctrl/Src/dcc_auto.d ./DCCCctrl/Src/dcc_auto.o ./DCCCctrl/Src/dcc_auto.su ./DCCCctrl/Src/dcc_tx.d ./DCCCctrl/Src/dcc_tx.o ./DCCCctrl/Src/dcc_tx.su ./DCCCctrl/Src/dcc_tx_hw.d ./DCCCctrl/Src/dcc_tx_hw.o ./DCCCctrl/Src/dcc_tx_hw.su ./DCCCctrl/Src/dccctrl_cmdi.d ./DCCCctrl/Src/dccctrl_cmdi.o ./DCCCctrl/Src/dccctrl_cmdi.su ./DCCCctrl/Src/dccctrl_config.d ./DCCCctrl/Src/dccctrl_config.o ./DCCCctrl/Src/dccctrl_config.su ./DCCCctrl/Src/dccctrl_timing.d ./DCCCctrl/Src/dccctrl_timing.o ./DCCCctrl/Src/dccctrl_timing.su ./DCCCctrl/Src/sx_tx.d ./DCCCctrl/Src/sx_tx.o ./DCCCctrl/Src/sx_tx.su

.PHONY: clean-DCCCctrl-2f-Src

