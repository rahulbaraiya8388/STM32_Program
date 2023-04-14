################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/SHT21/sht2x_for_stm32_hal.c 

OBJS += \
./Drivers/BSP/SHT21/sht2x_for_stm32_hal.o 

C_DEPS += \
./Drivers/BSP/SHT21/sht2x_for_stm32_hal.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/SHT21/%.o Drivers/BSP/SHT21/%.su: ../Drivers/BSP/SHT21/%.c Drivers/BSP/SHT21/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L152xC -c -I../Core/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/CMSIS/Include -I"D:/STM32_Projects/Workspace/temp_humi/Drivers/BSP/SHT21" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-SHT21

clean-Drivers-2f-BSP-2f-SHT21:
	-$(RM) ./Drivers/BSP/SHT21/sht2x_for_stm32_hal.d ./Drivers/BSP/SHT21/sht2x_for_stm32_hal.o ./Drivers/BSP/SHT21/sht2x_for_stm32_hal.su

.PHONY: clean-Drivers-2f-BSP-2f-SHT21

