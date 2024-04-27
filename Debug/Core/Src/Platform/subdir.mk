################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Platform/platform.c \
../Core/Src/Platform/platform_cf2.c \
../Core/Src/Platform/platform_stm32f4.c 

OBJS += \
./Core/Src/Platform/platform.o \
./Core/Src/Platform/platform_cf2.o \
./Core/Src/Platform/platform_stm32f4.o 

C_DEPS += \
./Core/Src/Platform/platform.d \
./Core/Src/Platform/platform_cf2.d \
./Core/Src/Platform/platform_stm32f4.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Platform/%.o Core/Src/Platform/%.su Core/Src/Platform/%.cyclo: ../Core/Src/Platform/%.c Core/Src/Platform/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -D__FPU_PRESENT -DUSE_HAL_DRIVER -DCRAZYFLIE_FW -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/swift/workspace/project/swiftF4/Core/Inc/Algo" -I"/home/swift/workspace/project/swiftF4/Core/Inc/App" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Comms" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Drivers" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Interface" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Platform" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Platform" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Config" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/swift/workspace/project/swiftF4/Core/Inc/Utils" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/ARM/DSP/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Platform

clean-Core-2f-Src-2f-Platform:
	-$(RM) ./Core/Src/Platform/platform.cyclo ./Core/Src/Platform/platform.d ./Core/Src/Platform/platform.o ./Core/Src/Platform/platform.su ./Core/Src/Platform/platform_cf2.cyclo ./Core/Src/Platform/platform_cf2.d ./Core/Src/Platform/platform_cf2.o ./Core/Src/Platform/platform_cf2.su ./Core/Src/Platform/platform_stm32f4.cyclo ./Core/Src/Platform/platform_stm32f4.d ./Core/Src/Platform/platform_stm32f4.o ./Core/Src/Platform/platform_stm32f4.su

.PHONY: clean-Core-2f-Src-2f-Platform

