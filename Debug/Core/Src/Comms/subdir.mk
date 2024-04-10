################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Comms/comm.c \
../Core/Src/Comms/crtp.c \
../Core/Src/Comms/crtp_commander.c \
../Core/Src/Comms/crtp_commander_generic.c \
../Core/Src/Comms/crtp_commander_high_level.c \
../Core/Src/Comms/crtp_commander_rpyt.c \
../Core/Src/Comms/crtp_localization_service.c \
../Core/Src/Comms/crtpservice.c \
../Core/Src/Comms/platformservice.c \
../Core/Src/Comms/radiolink.c \
../Core/Src/Comms/syslink.c 

OBJS += \
./Core/Src/Comms/comm.o \
./Core/Src/Comms/crtp.o \
./Core/Src/Comms/crtp_commander.o \
./Core/Src/Comms/crtp_commander_generic.o \
./Core/Src/Comms/crtp_commander_high_level.o \
./Core/Src/Comms/crtp_commander_rpyt.o \
./Core/Src/Comms/crtp_localization_service.o \
./Core/Src/Comms/crtpservice.o \
./Core/Src/Comms/platformservice.o \
./Core/Src/Comms/radiolink.o \
./Core/Src/Comms/syslink.o 

C_DEPS += \
./Core/Src/Comms/comm.d \
./Core/Src/Comms/crtp.d \
./Core/Src/Comms/crtp_commander.d \
./Core/Src/Comms/crtp_commander_generic.d \
./Core/Src/Comms/crtp_commander_high_level.d \
./Core/Src/Comms/crtp_commander_rpyt.d \
./Core/Src/Comms/crtp_localization_service.d \
./Core/Src/Comms/crtpservice.d \
./Core/Src/Comms/platformservice.d \
./Core/Src/Comms/radiolink.d \
./Core/Src/Comms/syslink.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Comms/%.o Core/Src/Comms/%.su Core/Src/Comms/%.cyclo: ../Core/Src/Comms/%.c Core/Src/Comms/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -D__FPU_PRESENT -DUSE_HAL_DRIVER -DCRAZYFLIE_FW -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/swift/workspace/project/swiftF4/Core/Inc/Algo" -I"/home/swift/workspace/project/swiftF4/Core/Inc/App" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Comms" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Drivers" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Interface" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Platform" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Platform" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Config" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/swift/workspace/project/swiftF4/Core/Inc/Utils" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Comms

clean-Core-2f-Src-2f-Comms:
	-$(RM) ./Core/Src/Comms/comm.cyclo ./Core/Src/Comms/comm.d ./Core/Src/Comms/comm.o ./Core/Src/Comms/comm.su ./Core/Src/Comms/crtp.cyclo ./Core/Src/Comms/crtp.d ./Core/Src/Comms/crtp.o ./Core/Src/Comms/crtp.su ./Core/Src/Comms/crtp_commander.cyclo ./Core/Src/Comms/crtp_commander.d ./Core/Src/Comms/crtp_commander.o ./Core/Src/Comms/crtp_commander.su ./Core/Src/Comms/crtp_commander_generic.cyclo ./Core/Src/Comms/crtp_commander_generic.d ./Core/Src/Comms/crtp_commander_generic.o ./Core/Src/Comms/crtp_commander_generic.su ./Core/Src/Comms/crtp_commander_high_level.cyclo ./Core/Src/Comms/crtp_commander_high_level.d ./Core/Src/Comms/crtp_commander_high_level.o ./Core/Src/Comms/crtp_commander_high_level.su ./Core/Src/Comms/crtp_commander_rpyt.cyclo ./Core/Src/Comms/crtp_commander_rpyt.d ./Core/Src/Comms/crtp_commander_rpyt.o ./Core/Src/Comms/crtp_commander_rpyt.su ./Core/Src/Comms/crtp_localization_service.cyclo ./Core/Src/Comms/crtp_localization_service.d ./Core/Src/Comms/crtp_localization_service.o ./Core/Src/Comms/crtp_localization_service.su ./Core/Src/Comms/crtpservice.cyclo ./Core/Src/Comms/crtpservice.d ./Core/Src/Comms/crtpservice.o ./Core/Src/Comms/crtpservice.su ./Core/Src/Comms/platformservice.cyclo ./Core/Src/Comms/platformservice.d ./Core/Src/Comms/platformservice.o ./Core/Src/Comms/platformservice.su ./Core/Src/Comms/radiolink.cyclo ./Core/Src/Comms/radiolink.d ./Core/Src/Comms/radiolink.o ./Core/Src/Comms/radiolink.su ./Core/Src/Comms/syslink.cyclo ./Core/Src/Comms/syslink.d ./Core/Src/Comms/syslink.o ./Core/Src/Comms/syslink.su

.PHONY: clean-Core-2f-Src-2f-Comms

