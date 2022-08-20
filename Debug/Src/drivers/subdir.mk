################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/drivers/led.c \
../Src/drivers/uart.c 

OBJS += \
./Src/drivers/led.o \
./Src/drivers/uart.o 

C_DEPS += \
./Src/drivers/led.d \
./Src/drivers/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/drivers/%.o Src/drivers/%.su: ../Src/drivers/%.c Src/drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F303xE -c -I../Inc -I/Users/fabricebeya/Documents/Projects/rtos/uart/uart_driver/chip_headers/Device/ST/STM32F3xx/Include -I/Users/fabricebeya/Documents/Projects/rtos/uart/uart_driver/chip_headers/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-drivers

clean-Src-2f-drivers:
	-$(RM) ./Src/drivers/led.d ./Src/drivers/led.o ./Src/drivers/led.su ./Src/drivers/uart.d ./Src/drivers/uart.o ./Src/drivers/uart.su

.PHONY: clean-Src-2f-drivers

