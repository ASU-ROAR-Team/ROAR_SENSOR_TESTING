################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Adafruit_BNO055/Adafruit_BNO055.c \
../Adafruit_BNO055/IMU.c \
../Adafruit_BNO055/bno055.c 

OBJS += \
./Adafruit_BNO055/Adafruit_BNO055.o \
./Adafruit_BNO055/IMU.o \
./Adafruit_BNO055/bno055.o 

C_DEPS += \
./Adafruit_BNO055/Adafruit_BNO055.d \
./Adafruit_BNO055/IMU.d \
./Adafruit_BNO055/bno055.d 


# Each subdirectory must supply rules for building sources it contributes
Adafruit_BNO055/%.o Adafruit_BNO055/%.su Adafruit_BNO055/%.cyclo: ../Adafruit_BNO055/%.c Adafruit_BNO055/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Adafruit_BNO055

clean-Adafruit_BNO055:
	-$(RM) ./Adafruit_BNO055/Adafruit_BNO055.cyclo ./Adafruit_BNO055/Adafruit_BNO055.d ./Adafruit_BNO055/Adafruit_BNO055.o ./Adafruit_BNO055/Adafruit_BNO055.su ./Adafruit_BNO055/IMU.cyclo ./Adafruit_BNO055/IMU.d ./Adafruit_BNO055/IMU.o ./Adafruit_BNO055/IMU.su ./Adafruit_BNO055/bno055.cyclo ./Adafruit_BNO055/bno055.d ./Adafruit_BNO055/bno055.o ./Adafruit_BNO055/bno055.su

.PHONY: clean-Adafruit_BNO055

