################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../roboclaw/roboclaw.c 

OBJS += \
./roboclaw/roboclaw.o 

C_DEPS += \
./roboclaw/roboclaw.d 


# Each subdirectory must supply rules for building sources it contributes
roboclaw/%.o roboclaw/%.su roboclaw/%.cyclo: ../roboclaw/%.c roboclaw/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-roboclaw

clean-roboclaw:
	-$(RM) ./roboclaw/roboclaw.cyclo ./roboclaw/roboclaw.d ./roboclaw/roboclaw.o ./roboclaw/roboclaw.su

.PHONY: clean-roboclaw

