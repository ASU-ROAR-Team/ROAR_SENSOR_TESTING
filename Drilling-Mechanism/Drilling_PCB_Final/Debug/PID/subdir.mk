################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../PID/motor_control.c \
../PID/pid_controller.c 

OBJS += \
./PID/motor_control.o \
./PID/pid_controller.o 

C_DEPS += \
./PID/motor_control.d \
./PID/pid_controller.d 


# Each subdirectory must supply rules for building sources it contributes
PID/%.o PID/%.su PID/%.cyclo: ../PID/%.c PID/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-PID

clean-PID:
	-$(RM) ./PID/motor_control.cyclo ./PID/motor_control.d ./PID/motor_control.o ./PID/motor_control.su ./PID/pid_controller.cyclo ./PID/pid_controller.d ./PID/pid_controller.o ./PID/pid_controller.su

.PHONY: clean-PID

