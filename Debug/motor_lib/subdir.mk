################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../motor_lib/Pwm_lib.c \
../motor_lib/car.c \
../motor_lib/motor_lib.c 

OBJS += \
./motor_lib/Pwm_lib.o \
./motor_lib/car.o \
./motor_lib/motor_lib.o 

C_DEPS += \
./motor_lib/Pwm_lib.d \
./motor_lib/car.d \
./motor_lib/motor_lib.d 


# Each subdirectory must supply rules for building sources it contributes
motor_lib/%.o motor_lib/%.su motor_lib/%.cyclo: ../motor_lib/%.c motor_lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/user/STM32CubeIDE/workspace_1.16.1/key_pad/Keypad_Lib" -I"D:/user/STM32CubeIDE/workspace_1.16.1/key_pad/motor_lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-motor_lib

clean-motor_lib:
	-$(RM) ./motor_lib/Pwm_lib.cyclo ./motor_lib/Pwm_lib.d ./motor_lib/Pwm_lib.o ./motor_lib/Pwm_lib.su ./motor_lib/car.cyclo ./motor_lib/car.d ./motor_lib/car.o ./motor_lib/car.su ./motor_lib/motor_lib.cyclo ./motor_lib/motor_lib.d ./motor_lib/motor_lib.o ./motor_lib/motor_lib.su

.PHONY: clean-motor_lib

