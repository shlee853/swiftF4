################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Drivers/ak8963.c \
../Core/Src/Drivers/bmi088_accel.c \
../Core/Src/Drivers/bmi088_fifo.c \
../Core/Src/Drivers/bmi088_gyro.c \
../Core/Src/Drivers/bmp280.c \
../Core/Src/Drivers/bmp3.c \
../Core/Src/Drivers/bootloader.c \
../Core/Src/Drivers/bstdr_comm_support.c \
../Core/Src/Drivers/buzzer.c \
../Core/Src/Drivers/i2c_drv.c \
../Core/Src/Drivers/i2cdev.c \
../Core/Src/Drivers/led.c \
../Core/Src/Drivers/ledseq.c \
../Core/Src/Drivers/lps25h.c \
../Core/Src/Drivers/mem.c \
../Core/Src/Drivers/motors.c \
../Core/Src/Drivers/motors_def.c \
../Core/Src/Drivers/mpu6500.c \
../Core/Src/Drivers/ow_common.c \
../Core/Src/Drivers/ow_none.c \
../Core/Src/Drivers/pm_stm32f4.c \
../Core/Src/Drivers/power_distribution_quadrotor.c \
../Core/Src/Drivers/sensfusion6.c \
../Core/Src/Drivers/sensors.c \
../Core/Src/Drivers/sensors_bmi088_bmp388.c \
../Core/Src/Drivers/sensors_bmi088_i2c.c \
../Core/Src/Drivers/sensors_bosch.c \
../Core/Src/Drivers/sensors_mpu9250_bmp280.c \
../Core/Src/Drivers/sound_cf2.c \
../Core/Src/Drivers/storage.c \
../Core/Src/Drivers/system.c \
../Core/Src/Drivers/uart_syslink.c \
../Core/Src/Drivers/usb.c \
../Core/Src/Drivers/usblink.c \
../Core/Src/Drivers/vcp_esc_passthrough.c 

OBJS += \
./Core/Src/Drivers/ak8963.o \
./Core/Src/Drivers/bmi088_accel.o \
./Core/Src/Drivers/bmi088_fifo.o \
./Core/Src/Drivers/bmi088_gyro.o \
./Core/Src/Drivers/bmp280.o \
./Core/Src/Drivers/bmp3.o \
./Core/Src/Drivers/bootloader.o \
./Core/Src/Drivers/bstdr_comm_support.o \
./Core/Src/Drivers/buzzer.o \
./Core/Src/Drivers/i2c_drv.o \
./Core/Src/Drivers/i2cdev.o \
./Core/Src/Drivers/led.o \
./Core/Src/Drivers/ledseq.o \
./Core/Src/Drivers/lps25h.o \
./Core/Src/Drivers/mem.o \
./Core/Src/Drivers/motors.o \
./Core/Src/Drivers/motors_def.o \
./Core/Src/Drivers/mpu6500.o \
./Core/Src/Drivers/ow_common.o \
./Core/Src/Drivers/ow_none.o \
./Core/Src/Drivers/pm_stm32f4.o \
./Core/Src/Drivers/power_distribution_quadrotor.o \
./Core/Src/Drivers/sensfusion6.o \
./Core/Src/Drivers/sensors.o \
./Core/Src/Drivers/sensors_bmi088_bmp388.o \
./Core/Src/Drivers/sensors_bmi088_i2c.o \
./Core/Src/Drivers/sensors_bosch.o \
./Core/Src/Drivers/sensors_mpu9250_bmp280.o \
./Core/Src/Drivers/sound_cf2.o \
./Core/Src/Drivers/storage.o \
./Core/Src/Drivers/system.o \
./Core/Src/Drivers/uart_syslink.o \
./Core/Src/Drivers/usb.o \
./Core/Src/Drivers/usblink.o \
./Core/Src/Drivers/vcp_esc_passthrough.o 

C_DEPS += \
./Core/Src/Drivers/ak8963.d \
./Core/Src/Drivers/bmi088_accel.d \
./Core/Src/Drivers/bmi088_fifo.d \
./Core/Src/Drivers/bmi088_gyro.d \
./Core/Src/Drivers/bmp280.d \
./Core/Src/Drivers/bmp3.d \
./Core/Src/Drivers/bootloader.d \
./Core/Src/Drivers/bstdr_comm_support.d \
./Core/Src/Drivers/buzzer.d \
./Core/Src/Drivers/i2c_drv.d \
./Core/Src/Drivers/i2cdev.d \
./Core/Src/Drivers/led.d \
./Core/Src/Drivers/ledseq.d \
./Core/Src/Drivers/lps25h.d \
./Core/Src/Drivers/mem.d \
./Core/Src/Drivers/motors.d \
./Core/Src/Drivers/motors_def.d \
./Core/Src/Drivers/mpu6500.d \
./Core/Src/Drivers/ow_common.d \
./Core/Src/Drivers/ow_none.d \
./Core/Src/Drivers/pm_stm32f4.d \
./Core/Src/Drivers/power_distribution_quadrotor.d \
./Core/Src/Drivers/sensfusion6.d \
./Core/Src/Drivers/sensors.d \
./Core/Src/Drivers/sensors_bmi088_bmp388.d \
./Core/Src/Drivers/sensors_bmi088_i2c.d \
./Core/Src/Drivers/sensors_bosch.d \
./Core/Src/Drivers/sensors_mpu9250_bmp280.d \
./Core/Src/Drivers/sound_cf2.d \
./Core/Src/Drivers/storage.d \
./Core/Src/Drivers/system.d \
./Core/Src/Drivers/uart_syslink.d \
./Core/Src/Drivers/usb.d \
./Core/Src/Drivers/usblink.d \
./Core/Src/Drivers/vcp_esc_passthrough.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Drivers/%.o Core/Src/Drivers/%.su Core/Src/Drivers/%.cyclo: ../Core/Src/Drivers/%.c Core/Src/Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -D__FPU_PRESENT -DUSE_HAL_DRIVER -DCRAZYFLIE_FW -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/swift/workspace/project/swiftF4/Core/Inc/Algo" -I"/home/swift/workspace/project/swiftF4/Core/Inc/App" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Comms" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Drivers" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Interface" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Platform" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Platform" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Config" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/swift/workspace/project/swiftF4/Core/Inc/Utils" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Drivers

clean-Core-2f-Src-2f-Drivers:
	-$(RM) ./Core/Src/Drivers/ak8963.cyclo ./Core/Src/Drivers/ak8963.d ./Core/Src/Drivers/ak8963.o ./Core/Src/Drivers/ak8963.su ./Core/Src/Drivers/bmi088_accel.cyclo ./Core/Src/Drivers/bmi088_accel.d ./Core/Src/Drivers/bmi088_accel.o ./Core/Src/Drivers/bmi088_accel.su ./Core/Src/Drivers/bmi088_fifo.cyclo ./Core/Src/Drivers/bmi088_fifo.d ./Core/Src/Drivers/bmi088_fifo.o ./Core/Src/Drivers/bmi088_fifo.su ./Core/Src/Drivers/bmi088_gyro.cyclo ./Core/Src/Drivers/bmi088_gyro.d ./Core/Src/Drivers/bmi088_gyro.o ./Core/Src/Drivers/bmi088_gyro.su ./Core/Src/Drivers/bmp280.cyclo ./Core/Src/Drivers/bmp280.d ./Core/Src/Drivers/bmp280.o ./Core/Src/Drivers/bmp280.su ./Core/Src/Drivers/bmp3.cyclo ./Core/Src/Drivers/bmp3.d ./Core/Src/Drivers/bmp3.o ./Core/Src/Drivers/bmp3.su ./Core/Src/Drivers/bootloader.cyclo ./Core/Src/Drivers/bootloader.d ./Core/Src/Drivers/bootloader.o ./Core/Src/Drivers/bootloader.su ./Core/Src/Drivers/bstdr_comm_support.cyclo ./Core/Src/Drivers/bstdr_comm_support.d ./Core/Src/Drivers/bstdr_comm_support.o ./Core/Src/Drivers/bstdr_comm_support.su ./Core/Src/Drivers/buzzer.cyclo ./Core/Src/Drivers/buzzer.d ./Core/Src/Drivers/buzzer.o ./Core/Src/Drivers/buzzer.su ./Core/Src/Drivers/i2c_drv.cyclo ./Core/Src/Drivers/i2c_drv.d ./Core/Src/Drivers/i2c_drv.o ./Core/Src/Drivers/i2c_drv.su ./Core/Src/Drivers/i2cdev.cyclo ./Core/Src/Drivers/i2cdev.d ./Core/Src/Drivers/i2cdev.o ./Core/Src/Drivers/i2cdev.su ./Core/Src/Drivers/led.cyclo ./Core/Src/Drivers/led.d ./Core/Src/Drivers/led.o ./Core/Src/Drivers/led.su ./Core/Src/Drivers/ledseq.cyclo ./Core/Src/Drivers/ledseq.d ./Core/Src/Drivers/ledseq.o ./Core/Src/Drivers/ledseq.su ./Core/Src/Drivers/lps25h.cyclo ./Core/Src/Drivers/lps25h.d ./Core/Src/Drivers/lps25h.o ./Core/Src/Drivers/lps25h.su ./Core/Src/Drivers/mem.cyclo ./Core/Src/Drivers/mem.d ./Core/Src/Drivers/mem.o ./Core/Src/Drivers/mem.su ./Core/Src/Drivers/motors.cyclo ./Core/Src/Drivers/motors.d ./Core/Src/Drivers/motors.o ./Core/Src/Drivers/motors.su ./Core/Src/Drivers/motors_def.cyclo ./Core/Src/Drivers/motors_def.d ./Core/Src/Drivers/motors_def.o ./Core/Src/Drivers/motors_def.su ./Core/Src/Drivers/mpu6500.cyclo ./Core/Src/Drivers/mpu6500.d ./Core/Src/Drivers/mpu6500.o ./Core/Src/Drivers/mpu6500.su ./Core/Src/Drivers/ow_common.cyclo ./Core/Src/Drivers/ow_common.d ./Core/Src/Drivers/ow_common.o ./Core/Src/Drivers/ow_common.su ./Core/Src/Drivers/ow_none.cyclo ./Core/Src/Drivers/ow_none.d ./Core/Src/Drivers/ow_none.o ./Core/Src/Drivers/ow_none.su ./Core/Src/Drivers/pm_stm32f4.cyclo ./Core/Src/Drivers/pm_stm32f4.d ./Core/Src/Drivers/pm_stm32f4.o ./Core/Src/Drivers/pm_stm32f4.su ./Core/Src/Drivers/power_distribution_quadrotor.cyclo ./Core/Src/Drivers/power_distribution_quadrotor.d ./Core/Src/Drivers/power_distribution_quadrotor.o ./Core/Src/Drivers/power_distribution_quadrotor.su ./Core/Src/Drivers/sensfusion6.cyclo ./Core/Src/Drivers/sensfusion6.d ./Core/Src/Drivers/sensfusion6.o ./Core/Src/Drivers/sensfusion6.su ./Core/Src/Drivers/sensors.cyclo ./Core/Src/Drivers/sensors.d ./Core/Src/Drivers/sensors.o ./Core/Src/Drivers/sensors.su ./Core/Src/Drivers/sensors_bmi088_bmp388.cyclo ./Core/Src/Drivers/sensors_bmi088_bmp388.d ./Core/Src/Drivers/sensors_bmi088_bmp388.o ./Core/Src/Drivers/sensors_bmi088_bmp388.su ./Core/Src/Drivers/sensors_bmi088_i2c.cyclo ./Core/Src/Drivers/sensors_bmi088_i2c.d ./Core/Src/Drivers/sensors_bmi088_i2c.o ./Core/Src/Drivers/sensors_bmi088_i2c.su ./Core/Src/Drivers/sensors_bosch.cyclo ./Core/Src/Drivers/sensors_bosch.d ./Core/Src/Drivers/sensors_bosch.o ./Core/Src/Drivers/sensors_bosch.su ./Core/Src/Drivers/sensors_mpu9250_bmp280.cyclo ./Core/Src/Drivers/sensors_mpu9250_bmp280.d ./Core/Src/Drivers/sensors_mpu9250_bmp280.o ./Core/Src/Drivers/sensors_mpu9250_bmp280.su ./Core/Src/Drivers/sound_cf2.cyclo ./Core/Src/Drivers/sound_cf2.d ./Core/Src/Drivers/sound_cf2.o ./Core/Src/Drivers/sound_cf2.su ./Core/Src/Drivers/storage.cyclo ./Core/Src/Drivers/storage.d ./Core/Src/Drivers/storage.o ./Core/Src/Drivers/storage.su ./Core/Src/Drivers/system.cyclo ./Core/Src/Drivers/system.d ./Core/Src/Drivers/system.o ./Core/Src/Drivers/system.su ./Core/Src/Drivers/uart_syslink.cyclo ./Core/Src/Drivers/uart_syslink.d ./Core/Src/Drivers/uart_syslink.o ./Core/Src/Drivers/uart_syslink.su ./Core/Src/Drivers/usb.cyclo ./Core/Src/Drivers/usb.d ./Core/Src/Drivers/usb.o ./Core/Src/Drivers/usb.su ./Core/Src/Drivers/usblink.cyclo ./Core/Src/Drivers/usblink.d ./Core/Src/Drivers/usblink.o ./Core/Src/Drivers/usblink.su ./Core/Src/Drivers/vcp_esc_passthrough.cyclo ./Core/Src/Drivers/vcp_esc_passthrough.d ./Core/Src/Drivers/vcp_esc_passthrough.o ./Core/Src/Drivers/vcp_esc_passthrough.su

.PHONY: clean-Core-2f-Src-2f-Drivers

