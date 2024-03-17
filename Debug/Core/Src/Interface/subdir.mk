################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Interface/attitude_pid_controller.c \
../Core/Src/Interface/collision_avoidance.c \
../Core/Src/Interface/commander.c \
../Core/Src/Interface/controller.c \
../Core/Src/Interface/controller_brescianini.c \
../Core/Src/Interface/controller_indi.c \
../Core/Src/Interface/controller_mellinger.c \
../Core/Src/Interface/controller_pid.c \
../Core/Src/Interface/deck.c \
../Core/Src/Interface/deck_analog.c \
../Core/Src/Interface/deck_constants.c \
../Core/Src/Interface/deck_digital.c \
../Core/Src/Interface/deck_drivers.c \
../Core/Src/Interface/deck_info.c \
../Core/Src/Interface/deck_memory.c \
../Core/Src/Interface/deck_spi.c \
../Core/Src/Interface/estimator.c \
../Core/Src/Interface/estimator_complementary.c \
../Core/Src/Interface/estimator_kalman.c \
../Core/Src/Interface/estimator_ukf.c \
../Core/Src/Interface/filter.c \
../Core/Src/Interface/kalman_core.c \
../Core/Src/Interface/kalman_supervisor.c \
../Core/Src/Interface/mm_absolute_height.c \
../Core/Src/Interface/mm_distance.c \
../Core/Src/Interface/mm_distance_robust.c \
../Core/Src/Interface/mm_flow.c \
../Core/Src/Interface/mm_pose.c \
../Core/Src/Interface/mm_position.c \
../Core/Src/Interface/mm_sweep_angles.c \
../Core/Src/Interface/mm_tdoa.c \
../Core/Src/Interface/mm_tdoa_robust.c \
../Core/Src/Interface/mm_tof.c \
../Core/Src/Interface/mm_yaw_error.c \
../Core/Src/Interface/outlierFilterLighthouse.c \
../Core/Src/Interface/outlierFilterTdoa.c \
../Core/Src/Interface/planner.c \
../Core/Src/Interface/position_controller_indi.c \
../Core/Src/Interface/position_controller_pid.c \
../Core/Src/Interface/position_estimator_altitude.c \
../Core/Src/Interface/pptraj.c \
../Core/Src/Interface/pptraj_compressed.c \
../Core/Src/Interface/rateSupervisor.c \
../Core/Src/Interface/stabilizer.c \
../Core/Src/Interface/supervisor.c \
../Core/Src/Interface/supervisor_state_machine.c 

OBJS += \
./Core/Src/Interface/attitude_pid_controller.o \
./Core/Src/Interface/collision_avoidance.o \
./Core/Src/Interface/commander.o \
./Core/Src/Interface/controller.o \
./Core/Src/Interface/controller_brescianini.o \
./Core/Src/Interface/controller_indi.o \
./Core/Src/Interface/controller_mellinger.o \
./Core/Src/Interface/controller_pid.o \
./Core/Src/Interface/deck.o \
./Core/Src/Interface/deck_analog.o \
./Core/Src/Interface/deck_constants.o \
./Core/Src/Interface/deck_digital.o \
./Core/Src/Interface/deck_drivers.o \
./Core/Src/Interface/deck_info.o \
./Core/Src/Interface/deck_memory.o \
./Core/Src/Interface/deck_spi.o \
./Core/Src/Interface/estimator.o \
./Core/Src/Interface/estimator_complementary.o \
./Core/Src/Interface/estimator_kalman.o \
./Core/Src/Interface/estimator_ukf.o \
./Core/Src/Interface/filter.o \
./Core/Src/Interface/kalman_core.o \
./Core/Src/Interface/kalman_supervisor.o \
./Core/Src/Interface/mm_absolute_height.o \
./Core/Src/Interface/mm_distance.o \
./Core/Src/Interface/mm_distance_robust.o \
./Core/Src/Interface/mm_flow.o \
./Core/Src/Interface/mm_pose.o \
./Core/Src/Interface/mm_position.o \
./Core/Src/Interface/mm_sweep_angles.o \
./Core/Src/Interface/mm_tdoa.o \
./Core/Src/Interface/mm_tdoa_robust.o \
./Core/Src/Interface/mm_tof.o \
./Core/Src/Interface/mm_yaw_error.o \
./Core/Src/Interface/outlierFilterLighthouse.o \
./Core/Src/Interface/outlierFilterTdoa.o \
./Core/Src/Interface/planner.o \
./Core/Src/Interface/position_controller_indi.o \
./Core/Src/Interface/position_controller_pid.o \
./Core/Src/Interface/position_estimator_altitude.o \
./Core/Src/Interface/pptraj.o \
./Core/Src/Interface/pptraj_compressed.o \
./Core/Src/Interface/rateSupervisor.o \
./Core/Src/Interface/stabilizer.o \
./Core/Src/Interface/supervisor.o \
./Core/Src/Interface/supervisor_state_machine.o 

C_DEPS += \
./Core/Src/Interface/attitude_pid_controller.d \
./Core/Src/Interface/collision_avoidance.d \
./Core/Src/Interface/commander.d \
./Core/Src/Interface/controller.d \
./Core/Src/Interface/controller_brescianini.d \
./Core/Src/Interface/controller_indi.d \
./Core/Src/Interface/controller_mellinger.d \
./Core/Src/Interface/controller_pid.d \
./Core/Src/Interface/deck.d \
./Core/Src/Interface/deck_analog.d \
./Core/Src/Interface/deck_constants.d \
./Core/Src/Interface/deck_digital.d \
./Core/Src/Interface/deck_drivers.d \
./Core/Src/Interface/deck_info.d \
./Core/Src/Interface/deck_memory.d \
./Core/Src/Interface/deck_spi.d \
./Core/Src/Interface/estimator.d \
./Core/Src/Interface/estimator_complementary.d \
./Core/Src/Interface/estimator_kalman.d \
./Core/Src/Interface/estimator_ukf.d \
./Core/Src/Interface/filter.d \
./Core/Src/Interface/kalman_core.d \
./Core/Src/Interface/kalman_supervisor.d \
./Core/Src/Interface/mm_absolute_height.d \
./Core/Src/Interface/mm_distance.d \
./Core/Src/Interface/mm_distance_robust.d \
./Core/Src/Interface/mm_flow.d \
./Core/Src/Interface/mm_pose.d \
./Core/Src/Interface/mm_position.d \
./Core/Src/Interface/mm_sweep_angles.d \
./Core/Src/Interface/mm_tdoa.d \
./Core/Src/Interface/mm_tdoa_robust.d \
./Core/Src/Interface/mm_tof.d \
./Core/Src/Interface/mm_yaw_error.d \
./Core/Src/Interface/outlierFilterLighthouse.d \
./Core/Src/Interface/outlierFilterTdoa.d \
./Core/Src/Interface/planner.d \
./Core/Src/Interface/position_controller_indi.d \
./Core/Src/Interface/position_controller_pid.d \
./Core/Src/Interface/position_estimator_altitude.d \
./Core/Src/Interface/pptraj.d \
./Core/Src/Interface/pptraj_compressed.d \
./Core/Src/Interface/rateSupervisor.d \
./Core/Src/Interface/stabilizer.d \
./Core/Src/Interface/supervisor.d \
./Core/Src/Interface/supervisor_state_machine.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Interface/%.o Core/Src/Interface/%.su Core/Src/Interface/%.cyclo: ../Core/Src/Interface/%.c Core/Src/Interface/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -D__FPU_PRESENT -DUSE_HAL_DRIVER -DCRAZYFLIE_FW -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/swift/workspace/project/swiftF4/Core/Inc/Algo" -I"/home/swift/workspace/project/swiftF4/Core/Inc/App" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Comms" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Drivers" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Interface" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Platform" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Platform" -I"/home/swift/workspace/project/swiftF4/Core/Inc/Config" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/swift/workspace/project/swiftF4/Core/Inc/Utils" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/ARM/DSP/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Interface

clean-Core-2f-Src-2f-Interface:
	-$(RM) ./Core/Src/Interface/attitude_pid_controller.cyclo ./Core/Src/Interface/attitude_pid_controller.d ./Core/Src/Interface/attitude_pid_controller.o ./Core/Src/Interface/attitude_pid_controller.su ./Core/Src/Interface/collision_avoidance.cyclo ./Core/Src/Interface/collision_avoidance.d ./Core/Src/Interface/collision_avoidance.o ./Core/Src/Interface/collision_avoidance.su ./Core/Src/Interface/commander.cyclo ./Core/Src/Interface/commander.d ./Core/Src/Interface/commander.o ./Core/Src/Interface/commander.su ./Core/Src/Interface/controller.cyclo ./Core/Src/Interface/controller.d ./Core/Src/Interface/controller.o ./Core/Src/Interface/controller.su ./Core/Src/Interface/controller_brescianini.cyclo ./Core/Src/Interface/controller_brescianini.d ./Core/Src/Interface/controller_brescianini.o ./Core/Src/Interface/controller_brescianini.su ./Core/Src/Interface/controller_indi.cyclo ./Core/Src/Interface/controller_indi.d ./Core/Src/Interface/controller_indi.o ./Core/Src/Interface/controller_indi.su ./Core/Src/Interface/controller_mellinger.cyclo ./Core/Src/Interface/controller_mellinger.d ./Core/Src/Interface/controller_mellinger.o ./Core/Src/Interface/controller_mellinger.su ./Core/Src/Interface/controller_pid.cyclo ./Core/Src/Interface/controller_pid.d ./Core/Src/Interface/controller_pid.o ./Core/Src/Interface/controller_pid.su ./Core/Src/Interface/deck.cyclo ./Core/Src/Interface/deck.d ./Core/Src/Interface/deck.o ./Core/Src/Interface/deck.su ./Core/Src/Interface/deck_analog.cyclo ./Core/Src/Interface/deck_analog.d ./Core/Src/Interface/deck_analog.o ./Core/Src/Interface/deck_analog.su ./Core/Src/Interface/deck_constants.cyclo ./Core/Src/Interface/deck_constants.d ./Core/Src/Interface/deck_constants.o ./Core/Src/Interface/deck_constants.su ./Core/Src/Interface/deck_digital.cyclo ./Core/Src/Interface/deck_digital.d ./Core/Src/Interface/deck_digital.o ./Core/Src/Interface/deck_digital.su ./Core/Src/Interface/deck_drivers.cyclo ./Core/Src/Interface/deck_drivers.d ./Core/Src/Interface/deck_drivers.o ./Core/Src/Interface/deck_drivers.su ./Core/Src/Interface/deck_info.cyclo ./Core/Src/Interface/deck_info.d ./Core/Src/Interface/deck_info.o ./Core/Src/Interface/deck_info.su ./Core/Src/Interface/deck_memory.cyclo ./Core/Src/Interface/deck_memory.d ./Core/Src/Interface/deck_memory.o ./Core/Src/Interface/deck_memory.su ./Core/Src/Interface/deck_spi.cyclo ./Core/Src/Interface/deck_spi.d ./Core/Src/Interface/deck_spi.o ./Core/Src/Interface/deck_spi.su ./Core/Src/Interface/estimator.cyclo ./Core/Src/Interface/estimator.d ./Core/Src/Interface/estimator.o ./Core/Src/Interface/estimator.su ./Core/Src/Interface/estimator_complementary.cyclo ./Core/Src/Interface/estimator_complementary.d ./Core/Src/Interface/estimator_complementary.o ./Core/Src/Interface/estimator_complementary.su ./Core/Src/Interface/estimator_kalman.cyclo ./Core/Src/Interface/estimator_kalman.d ./Core/Src/Interface/estimator_kalman.o ./Core/Src/Interface/estimator_kalman.su ./Core/Src/Interface/estimator_ukf.cyclo ./Core/Src/Interface/estimator_ukf.d ./Core/Src/Interface/estimator_ukf.o ./Core/Src/Interface/estimator_ukf.su ./Core/Src/Interface/filter.cyclo ./Core/Src/Interface/filter.d ./Core/Src/Interface/filter.o ./Core/Src/Interface/filter.su ./Core/Src/Interface/kalman_core.cyclo ./Core/Src/Interface/kalman_core.d ./Core/Src/Interface/kalman_core.o ./Core/Src/Interface/kalman_core.su ./Core/Src/Interface/kalman_supervisor.cyclo ./Core/Src/Interface/kalman_supervisor.d ./Core/Src/Interface/kalman_supervisor.o ./Core/Src/Interface/kalman_supervisor.su ./Core/Src/Interface/mm_absolute_height.cyclo ./Core/Src/Interface/mm_absolute_height.d ./Core/Src/Interface/mm_absolute_height.o ./Core/Src/Interface/mm_absolute_height.su ./Core/Src/Interface/mm_distance.cyclo ./Core/Src/Interface/mm_distance.d ./Core/Src/Interface/mm_distance.o ./Core/Src/Interface/mm_distance.su ./Core/Src/Interface/mm_distance_robust.cyclo ./Core/Src/Interface/mm_distance_robust.d ./Core/Src/Interface/mm_distance_robust.o ./Core/Src/Interface/mm_distance_robust.su ./Core/Src/Interface/mm_flow.cyclo ./Core/Src/Interface/mm_flow.d ./Core/Src/Interface/mm_flow.o ./Core/Src/Interface/mm_flow.su ./Core/Src/Interface/mm_pose.cyclo ./Core/Src/Interface/mm_pose.d ./Core/Src/Interface/mm_pose.o ./Core/Src/Interface/mm_pose.su ./Core/Src/Interface/mm_position.cyclo ./Core/Src/Interface/mm_position.d ./Core/Src/Interface/mm_position.o ./Core/Src/Interface/mm_position.su ./Core/Src/Interface/mm_sweep_angles.cyclo ./Core/Src/Interface/mm_sweep_angles.d ./Core/Src/Interface/mm_sweep_angles.o ./Core/Src/Interface/mm_sweep_angles.su ./Core/Src/Interface/mm_tdoa.cyclo ./Core/Src/Interface/mm_tdoa.d ./Core/Src/Interface/mm_tdoa.o ./Core/Src/Interface/mm_tdoa.su ./Core/Src/Interface/mm_tdoa_robust.cyclo ./Core/Src/Interface/mm_tdoa_robust.d ./Core/Src/Interface/mm_tdoa_robust.o ./Core/Src/Interface/mm_tdoa_robust.su ./Core/Src/Interface/mm_tof.cyclo ./Core/Src/Interface/mm_tof.d ./Core/Src/Interface/mm_tof.o ./Core/Src/Interface/mm_tof.su ./Core/Src/Interface/mm_yaw_error.cyclo ./Core/Src/Interface/mm_yaw_error.d ./Core/Src/Interface/mm_yaw_error.o ./Core/Src/Interface/mm_yaw_error.su ./Core/Src/Interface/outlierFilterLighthouse.cyclo ./Core/Src/Interface/outlierFilterLighthouse.d ./Core/Src/Interface/outlierFilterLighthouse.o ./Core/Src/Interface/outlierFilterLighthouse.su ./Core/Src/Interface/outlierFilterTdoa.cyclo ./Core/Src/Interface/outlierFilterTdoa.d ./Core/Src/Interface/outlierFilterTdoa.o ./Core/Src/Interface/outlierFilterTdoa.su ./Core/Src/Interface/planner.cyclo ./Core/Src/Interface/planner.d ./Core/Src/Interface/planner.o ./Core/Src/Interface/planner.su ./Core/Src/Interface/position_controller_indi.cyclo ./Core/Src/Interface/position_controller_indi.d ./Core/Src/Interface/position_controller_indi.o ./Core/Src/Interface/position_controller_indi.su ./Core/Src/Interface/position_controller_pid.cyclo ./Core/Src/Interface/position_controller_pid.d
	-$(RM) ./Core/Src/Interface/position_controller_pid.o ./Core/Src/Interface/position_controller_pid.su ./Core/Src/Interface/position_estimator_altitude.cyclo ./Core/Src/Interface/position_estimator_altitude.d ./Core/Src/Interface/position_estimator_altitude.o ./Core/Src/Interface/position_estimator_altitude.su ./Core/Src/Interface/pptraj.cyclo ./Core/Src/Interface/pptraj.d ./Core/Src/Interface/pptraj.o ./Core/Src/Interface/pptraj.su ./Core/Src/Interface/pptraj_compressed.cyclo ./Core/Src/Interface/pptraj_compressed.d ./Core/Src/Interface/pptraj_compressed.o ./Core/Src/Interface/pptraj_compressed.su ./Core/Src/Interface/rateSupervisor.cyclo ./Core/Src/Interface/rateSupervisor.d ./Core/Src/Interface/rateSupervisor.o ./Core/Src/Interface/rateSupervisor.su ./Core/Src/Interface/stabilizer.cyclo ./Core/Src/Interface/stabilizer.d ./Core/Src/Interface/stabilizer.o ./Core/Src/Interface/stabilizer.su ./Core/Src/Interface/supervisor.cyclo ./Core/Src/Interface/supervisor.d ./Core/Src/Interface/supervisor.o ./Core/Src/Interface/supervisor.su ./Core/Src/Interface/supervisor_state_machine.cyclo ./Core/Src/Interface/supervisor_state_machine.d ./Core/Src/Interface/supervisor_state_machine.o ./Core/Src/Interface/supervisor_state_machine.su

.PHONY: clean-Core-2f-Src-2f-Interface

