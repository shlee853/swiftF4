################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Utils/SEGGER_RTT.c \
../Core/Src/Utils/SEGGER_RTT_printf.c 

OBJS += \
./Core/Src/Utils/SEGGER_RTT.o \
./Core/Src/Utils/SEGGER_RTT_printf.o 

C_DEPS += \
./Core/Src/Utils/SEGGER_RTT.d \
./Core/Src/Utils/SEGGER_RTT_printf.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Utils/%.o Core/Src/Utils/%.su Core/Src/Utils/%.cyclo: ../Core/Src/Utils/%.c Core/Src/Utils/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/swift/workspace/project/swiftF4/Core/Inc/Config" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/swift/workspace/project/swiftF4/Core/Inc/Utils" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Utils

clean-Core-2f-Src-2f-Utils:
	-$(RM) ./Core/Src/Utils/SEGGER_RTT.cyclo ./Core/Src/Utils/SEGGER_RTT.d ./Core/Src/Utils/SEGGER_RTT.o ./Core/Src/Utils/SEGGER_RTT.su ./Core/Src/Utils/SEGGER_RTT_printf.cyclo ./Core/Src/Utils/SEGGER_RTT_printf.d ./Core/Src/Utils/SEGGER_RTT_printf.o ./Core/Src/Utils/SEGGER_RTT_printf.su

.PHONY: clean-Core-2f-Src-2f-Utils

