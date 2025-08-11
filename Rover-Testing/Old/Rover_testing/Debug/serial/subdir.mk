################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../serial/Serial.c \
../serial/base_serial.c \
../serial/ringbuffer.c 

OBJS += \
./serial/Serial.o \
./serial/base_serial.o \
./serial/ringbuffer.o 

C_DEPS += \
./serial/Serial.d \
./serial/base_serial.d \
./serial/ringbuffer.d 


# Each subdirectory must supply rules for building sources it contributes
serial/%.o serial/%.su serial/%.cyclo: ../serial/%.c serial/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-serial

clean-serial:
	-$(RM) ./serial/Serial.cyclo ./serial/Serial.d ./serial/Serial.o ./serial/Serial.su ./serial/base_serial.cyclo ./serial/base_serial.d ./serial/base_serial.o ./serial/base_serial.su ./serial/ringbuffer.cyclo ./serial/ringbuffer.d ./serial/ringbuffer.o ./serial/ringbuffer.su

.PHONY: clean-serial

