################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Keypad_Lib/Keypad.c 

OBJS += \
./Keypad_Lib/Keypad.o 

C_DEPS += \
./Keypad_Lib/Keypad.d 


# Each subdirectory must supply rules for building sources it contributes
Keypad_Lib/%.o Keypad_Lib/%.su Keypad_Lib/%.cyclo: ../Keypad_Lib/%.c Keypad_Lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/user/STM32CubeIDE/workspace_1.16.1/key_pad/Keypad_Lib" -I"D:/user/STM32CubeIDE/workspace_1.16.1/key_pad/motor_lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Keypad_Lib

clean-Keypad_Lib:
	-$(RM) ./Keypad_Lib/Keypad.cyclo ./Keypad_Lib/Keypad.d ./Keypad_Lib/Keypad.o ./Keypad_Lib/Keypad.su

.PHONY: clean-Keypad_Lib

