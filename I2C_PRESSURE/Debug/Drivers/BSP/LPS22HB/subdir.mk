################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/LPS22HB/lps22hb_reg.c 

OBJS += \
./Drivers/BSP/LPS22HB/lps22hb_reg.o 

C_DEPS += \
./Drivers/BSP/LPS22HB/lps22hb_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/LPS22HB/%.o Drivers/BSP/LPS22HB/%.su: ../Drivers/BSP/LPS22HB/%.c Drivers/BSP/LPS22HB/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L152xC -c -I../Core/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/CMSIS/Include -I"D:/STM32_Projects/Workspace/I2C_PRESSURE/Drivers/BSP" -I"D:/STM32_Projects/Workspace/I2C_PRESSURE/Drivers/BSP/LPS22HB" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-LPS22HB

clean-Drivers-2f-BSP-2f-LPS22HB:
	-$(RM) ./Drivers/BSP/LPS22HB/lps22hb_reg.d ./Drivers/BSP/LPS22HB/lps22hb_reg.o ./Drivers/BSP/LPS22HB/lps22hb_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-LPS22HB

