################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/app_freertos.c \
../Core/Src/bsp_AT24C02.c \
../Core/Src/bsp_IIC.c \
../Core/Src/bsp_autotune.c \
../Core/Src/bsp_hlw8032.c \
../Core/Src/bsp_io.c \
../Core/Src/bsp_max31855_driver.c \
../Core/Src/bsp_pid.c \
../Core/Src/bsp_tm1652.c \
../Core/Src/gpio.c \
../Core/Src/main.c \
../Core/Src/stm32g0xx_hal_msp.c \
../Core/Src/stm32g0xx_hal_timebase_tim.c \
../Core/Src/stm32g0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g0xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/app_freertos.o \
./Core/Src/bsp_AT24C02.o \
./Core/Src/bsp_IIC.o \
./Core/Src/bsp_autotune.o \
./Core/Src/bsp_hlw8032.o \
./Core/Src/bsp_io.o \
./Core/Src/bsp_max31855_driver.o \
./Core/Src/bsp_pid.o \
./Core/Src/bsp_tm1652.o \
./Core/Src/gpio.o \
./Core/Src/main.o \
./Core/Src/stm32g0xx_hal_msp.o \
./Core/Src/stm32g0xx_hal_timebase_tim.o \
./Core/Src/stm32g0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g0xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/app_freertos.d \
./Core/Src/bsp_AT24C02.d \
./Core/Src/bsp_IIC.d \
./Core/Src/bsp_autotune.d \
./Core/Src/bsp_hlw8032.d \
./Core/Src/bsp_io.d \
./Core/Src/bsp_max31855_driver.d \
./Core/Src/bsp_pid.d \
./Core/Src/bsp_tm1652.d \
./Core/Src/gpio.d \
./Core/Src/main.d \
./Core/Src/stm32g0xx_hal_msp.d \
./Core/Src/stm32g0xx_hal_timebase_tim.d \
./Core/Src/stm32g0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g0xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G070xx '-DCMSIS_device_header=<stm32g0xx.h>' -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/app_freertos.cyclo ./Core/Src/app_freertos.d ./Core/Src/app_freertos.o ./Core/Src/app_freertos.su ./Core/Src/bsp_AT24C02.cyclo ./Core/Src/bsp_AT24C02.d ./Core/Src/bsp_AT24C02.o ./Core/Src/bsp_AT24C02.su ./Core/Src/bsp_IIC.cyclo ./Core/Src/bsp_IIC.d ./Core/Src/bsp_IIC.o ./Core/Src/bsp_IIC.su ./Core/Src/bsp_autotune.cyclo ./Core/Src/bsp_autotune.d ./Core/Src/bsp_autotune.o ./Core/Src/bsp_autotune.su ./Core/Src/bsp_hlw8032.cyclo ./Core/Src/bsp_hlw8032.d ./Core/Src/bsp_hlw8032.o ./Core/Src/bsp_hlw8032.su ./Core/Src/bsp_io.cyclo ./Core/Src/bsp_io.d ./Core/Src/bsp_io.o ./Core/Src/bsp_io.su ./Core/Src/bsp_max31855_driver.cyclo ./Core/Src/bsp_max31855_driver.d ./Core/Src/bsp_max31855_driver.o ./Core/Src/bsp_max31855_driver.su ./Core/Src/bsp_pid.cyclo ./Core/Src/bsp_pid.d ./Core/Src/bsp_pid.o ./Core/Src/bsp_pid.su ./Core/Src/bsp_tm1652.cyclo ./Core/Src/bsp_tm1652.d ./Core/Src/bsp_tm1652.o ./Core/Src/bsp_tm1652.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32g0xx_hal_msp.cyclo ./Core/Src/stm32g0xx_hal_msp.d ./Core/Src/stm32g0xx_hal_msp.o ./Core/Src/stm32g0xx_hal_msp.su ./Core/Src/stm32g0xx_hal_timebase_tim.cyclo ./Core/Src/stm32g0xx_hal_timebase_tim.d ./Core/Src/stm32g0xx_hal_timebase_tim.o ./Core/Src/stm32g0xx_hal_timebase_tim.su ./Core/Src/stm32g0xx_it.cyclo ./Core/Src/stm32g0xx_it.d ./Core/Src/stm32g0xx_it.o ./Core/Src/stm32g0xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g0xx.cyclo ./Core/Src/system_stm32g0xx.d ./Core/Src/system_stm32g0xx.o ./Core/Src/system_stm32g0xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

