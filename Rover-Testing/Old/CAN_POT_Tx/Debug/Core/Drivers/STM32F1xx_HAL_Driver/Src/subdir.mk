################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
../Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c \
../Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c \
../Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.c \
../Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
../Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
../Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c \
../Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
../Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
../Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
../Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
../Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
../Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
../Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c 

OBJS += \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.o \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.o \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.o \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.o \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.o \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.o \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.o \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.o \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.o \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.o \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.o \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.o \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.o \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.o 

C_DEPS += \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.d \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.d \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.d \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.d \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.d \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.d \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.d \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.d \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.d \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.d \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.d \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.d \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.d \
./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Drivers/STM32F1xx_HAL_Driver/Src/%.o Core/Drivers/STM32F1xx_HAL_Driver/Src/%.su Core/Drivers/STM32F1xx_HAL_Driver/Src/%.cyclo: ../Core/Drivers/STM32F1xx_HAL_Driver/Src/%.c Core/Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Drivers-2f-STM32F1xx_HAL_Driver-2f-Src

clean-Core-2f-Drivers-2f-STM32F1xx_HAL_Driver-2f-Src:
	-$(RM) ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.cyclo ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.d ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.o ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.su ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.cyclo ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.d ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.o ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.su ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.cyclo ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.d ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.o ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.su ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.cyclo ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.d ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.o ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.su ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.cyclo ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.d ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.o ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.su ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.cyclo ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.d ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.o ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.su ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.cyclo ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.d ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.o ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.su ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.cyclo ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.d ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.o ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.su ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.cyclo ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.d ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.o ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.su ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.cyclo ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.d ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.o ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.su ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.cyclo ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.d ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.o ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.su ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.cyclo ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.d ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.o ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.su ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.cyclo ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.d ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.o ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.su ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.cyclo ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.d ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.o ./Core/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.su

.PHONY: clean-Core-2f-Drivers-2f-STM32F1xx_HAL_Driver-2f-Src

